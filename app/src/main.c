#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/settings/settings.h>
#include <zephyr/drivers/hwinfo.h>

#include "mesh.h"
#include "board.h"
#include "sensor.h"



// states and state changes
static uint8_t dev_uuid[16] = { 0xef, 0xbe, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Provisioning
// ================================================================
static void attention_on(struct bt_mesh_model *model)
{
	printk("attention_on()\n");
    attn_led(true);
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("attention_off()\n");
    attn_led(false);
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static int provisioning_output_pin(bt_mesh_output_action_t action, uint32_t number) {
	printk("OOB Number: %u\n", number);
	return 0;
}

static void provisioning_complete(uint16_t net_idx, uint16_t addr) {
    printk("Provisioning completed\n");
    prov_led(true);
}

static void provisioning_reset(void)
{
    printk("Provisioning reset\n");
    prov_led(false);

	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

// provisioning properties and capabilities
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = provisioning_output_pin,
	.complete = provisioning_complete,
	.reset = provisioning_reset,
};

// Node Composition
// ================================================================

#define CONFIG_SERVER_MODEL 0
#define HEALTH_SERVER_MODEL 1
#define SENSOR_SERVER_MODEL 2
#define SENSOR_SETUP_SERVER_MODEL 3

static struct bt_mesh_model sig_models[];

// Health Server
// ========
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

// Sensor Server
// ========

// sensor server states
// NB: "The Sensor Cadence state may be not supported by sensors based on device properties referencing
//      non- scalar characteristics such as histograms or composite characteristics"
//
// note that the sensor state is split across the two models, Sensor Server and Sensor Setup Server
//
// all we want to do is to publish an unsolicited Sensor Status message whenever the connected occupancy sensor reports an event
//
// we might implement GET which requires the same status message as a response
static uint8_t sensor_descriptor;

/*
* Sensor data has to be marshalled using the rules specified in 4.2.14 of the mesh models specification
*    MPID - Marshalling Property ID
*      1-bit Format field 
*        Format A - 0b0 - a 4-bit Length field and an 11-bit Property ID field
*        Format B - 0b1 - a 7-bit Length field and a 16-bit Property ID field
*      4-bit or 7-bit Length of the Property Value field
*      11-bit or 16-bit Property ID
*    Raw Value
*      Raw Value field with a size and representation defined by the device property.
*/

/*
* Three mesh properties are included in status messages for this sensor:
*  Present Ambient Temperature property [0x004F] (Temperature 8 characteristic)
*    The Temperature 8 characteristic is used to represent a measure of temperature with a unit of 0.5 degree Celsius.
*    sint8
*    Unit is degree Celsius with a resolution of 0.5.
*    Minimum: -64.0
*    Maximum: 63.0
*    MPID=0b0_0001_000_0100_1111 (2 octets)
*    MPID=0b1_0000001_0000_0000_0100_1111 (2 octets)
*
*  Present Ambient Relative Humidity device property [0x0076] (Humidity characteristic)
*    The Humidity characteristic is a fixed-length structure containing a single Humidity field.
*    uint16
*    Unit is in percent with a resolution of 0.01 percent.
*    Allowed range is: 0.00 to 100.00
*    MPID=0b0_0010_000_0111_0110 (2 octets)
*    MPID=0b1_0000010_0000_0000_0111_0110 (2 octets)
*  
*  Air Pressure device property [0x0082] (Pressure characteristic)
*    The Pressure characteristic is used to represent pressure.
*    uint32
*    Unit is Pascals with a resolution of 0.1 Pa.
*    TODO: Allowed range
*    MPID=0b0_0100_000_1000_0010 (2 octets)
*    
*/

static int TEMP_VALUE_INX = 3;
static int HUMIDITY_VALUE_INX = 7;
static int PRESSURE_VALUE_INX = 9;
static int SENSOR_DATA_LEN = 9;
static uint8_t temp_press_humidity[9] = {
        // Temperature
		0x81, // MPID: 0b1_0000000 = Format B, length 1
		0x4F, // 16-bit property ID byte 1 of 2: 0b01001111 = Temperature 8 characteristic
		0x00, // 16-bit property ID byte 2 of 2: 0b00000000
		0x00, // 1 byte value
        // Humidity
		0x82, // MPID: 0b1_0000001 = Format B, length 2
		0x76, // 16-bit property ID byte 1 of 2: 0b01110110 = Humidity characteristic
		0x00, // 16-bit property ID byte 2 of 2: 0b00000000
        0xFF, // value 1 of 2
        0xFF, // value 2 of 2
        ////Pressure
        //0x20, // MPID: 0b00100000 = Format A, lenght 4
		//0x82, // 11-bit property ID (inc 3 bits from first octet): 0b01110110 = Humidity characteristic
        //0x00, // value 1 of 4
        //0x00, // value 2 of 4
        //0x00, // value 3 of 4
        //0x00, // value 4 of 4
};

// messages the model might publish
#define BT_MESH_MODEL_OP_SENSOR_STATUS BT_MESH_MODEL_OP_1(0x52)

static int update_sensor_status_pub(struct bt_mesh_model *mod);

// Sensor Server Setup
BT_MESH_MODEL_PUB_DEFINE(sensor_status_pub, update_sensor_status_pub, 16);

static int update_sensor_status_pub(struct bt_mesh_model *mod)
{
    // Fetch sensor data
    sample_sensor();

    int temp_int = (int) (sensor_readings.temp * 10);
    int press_units = (int) (sensor_readings.press * 10 * 1000);
    int humidity_units = (int) (sensor_readings.humidity * 100);

    printk("temp: %d.%1dC\tpress: %d.%1dPa\thumidity: %d.%2d\n", temp_int / 10, temp_int % 10, press_units / 10, press_units % 10, humidity_units / 100, humidity_units % 100);

    // Unit is degree Celsius with a resolution of 0.5
    int temp_units = (int) (temp_int * 2);
    temp_press_humidity[TEMP_VALUE_INX] = temp_units & 255;

    //// Unit is Pascal with a resolution of 0.1
    //temp_press_humidity[PRESSURE_VALUE_INX] = (press_units >> 24) & 255;
    //temp_press_humidity[PRESSURE_VALUE_INX+1] = (press_units >> 16) & 255;
    //temp_press_humidity[PRESSURE_VALUE_INX+2] = (press_units >> 8) & 255;
    //temp_press_humidity[PRESSURE_VALUE_INX+3] = (press_units) & 255;

    //// Unit is in percent with a resolution of 0.01 percent
    //temp_press_humidity[HUMIDITY_VALUE_INX] = (humidity_units >> 8) & 255;
    //temp_press_humidity[HUMIDITY_VALUE_INX+1] = (humidity_units) & 255;


    // Prepare pub buffer
    bt_mesh_model_msg_init(sensor_status_pub.msg, BT_MESH_MODEL_OP_SENSOR_STATUS);

    int i = 0;
    for (i=0; i < SENSOR_DATA_LEN; i++) {
        net_buf_simple_add_u8(sensor_status_pub.msg, temp_press_humidity[i]);
    }

    printk("status pub buffer\n");
    for (i=0; i < sensor_status_pub.msg->len; i++) {
        printk("%02x", sensor_status_pub.msg->data[i]);
    }
    printk("\n");

    return 0;
}

// handler functions for this model's operations. We only need handlers for those message types we might receive.
static void sensor_descriptor_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("sensor_descriptor_get - ignoring\n");
}

static int sensor_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buffer)
{
    // Fetch sensor data
    sample_sensor();

    int temp_int = (int) (sensor_readings.temp * 10);
    int press_units = (int) (sensor_readings.press * 10 * 1000);
    int humidity_units = (int) (sensor_readings.humidity * 100);

    printk("temp: %d.%1dC\tpress: %d.%1dPa\thumidity: %d.%2d\n", temp_int / 10, temp_int % 10, press_units / 10, press_units % 10, humidity_units / 100, humidity_units % 100);

    // Unit is degree Celsius with a resolution of 0.5
    int temp_units = (int) (temp_int * 2);
    temp_press_humidity[TEMP_VALUE_INX] = temp_units & 255;

    //// Unit is Pascal with a resolution of 0.1
    //temp_press_humidity[PRESSURE_VALUE_INX] = (press_units >> 24) & 255;
    //temp_press_humidity[PRESSURE_VALUE_INX+1] = (press_units >> 16) & 255;
    //temp_press_humidity[PRESSURE_VALUE_INX+2] = (press_units >> 8) & 255;
    //temp_press_humidity[PRESSURE_VALUE_INX+3] = (press_units) & 255;

    // Unit is in percent with a resolution of 0.01 percent
    temp_press_humidity[HUMIDITY_VALUE_INX] = (humidity_units >> 8) & 255;
    temp_press_humidity[HUMIDITY_VALUE_INX+1] = (humidity_units) & 255;


    // Prepare pub buffer
	BT_MESH_MODEL_BUF_DEFINE(buf, BT_MESH_MODEL_OP_SENSOR_STATUS, SENSOR_DATA_LEN);
    bt_mesh_model_msg_init(&buf, BT_MESH_MODEL_OP_SENSOR_STATUS);

    int i = 0;
    for (i=0; i < SENSOR_DATA_LEN; i++) {
        net_buf_simple_add_u8(&buf, temp_press_humidity[i]);
    }

	int err = bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
	if (err)
	{
		printk("ERROR publishing sensor status message (err %d)\n", err);
		if (err == -EADDRNOTAVAIL) {
			printk("Publish Address has not been configured\n");
		}
	}

    return 0;
}

static void sensor_column_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("sensor_column_get - ignoring\n");
}

static void sensor_series_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("sensor_series_get - ignoring\n");
}

// messages the model might receive
static const struct bt_mesh_model_op sensor_server_op[] = {
		{BT_MESH_MODEL_OP_2(0x82, 0x30), 0, sensor_descriptor_get},
		{BT_MESH_MODEL_OP_2(0x82, 0x31), 0, sensor_get},
		{BT_MESH_MODEL_OP_2(0x82, 0x32), 2, sensor_column_get},
		{BT_MESH_MODEL_OP_2(0x82, 0x33), 2, sensor_series_get},
		BT_MESH_MODEL_OP_END,
};

struct sensor_setting_t
{
	uint16_t sensor_property_id;
	uint16_t sensor_setting_property_id;
	uint8_t sensor_setting_access;
	uint8_t *sensor_raw;
};

static struct sensor_setting_t sensor_setting;

// handler functions for this model's operations. We only need handlers for those message types we might receive.
// we're not supporting the sensor cadence state so no handlers are required and any such messages will be ignored, per the spec (3.7.4.4).
static void sensor_settings_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("sensor_settings_get - ignoring\n");
}

static void sensor_setting_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("sensor_setting_get - ignoring\n");
}

static void sensor_setting_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("sensor_setting_set - ignoring\n");
}

static void sensor_setting_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("sensor_setting_set_unack - ignoring\n");
}

// operations supported by this model
static const struct bt_mesh_model_op sensor_setup_server_op[] = {
		{BT_MESH_MODEL_OP_2(0x82, 0x35), 2, sensor_settings_get},
		{BT_MESH_MODEL_OP_2(0x82, 0x36), 2, sensor_setting_get},
		{BT_MESH_MODEL_OP_1(0x59), 2, sensor_setting_set},
		{BT_MESH_MODEL_OP_1(0x5A), 2, sensor_setting_set_unack},
		BT_MESH_MODEL_OP_END,
};

// SIG models for the first element
static struct bt_mesh_model sig_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_server_op, &sensor_status_pub, NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SETUP_SRV, sensor_setup_server_op, NULL, NULL),
};

// First element
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

// Node composition
static struct bt_mesh_comp comp = {
    .cid = 0xFFFF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/**
 * mesh_reset_work(): reset mesh state and begin advertising
 * @work: ignored
 *
 */
void mesh_reset_work(struct k_work *work)
{
    bt_mesh_reset();
	printk("Mesh state is reset!\n");

	printk("Begin advertising!\n");
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

/**
 * bt_ready(): Callback for bt_mesh_init
 * @err: [TODO:description]
 *
 * [TODO:description]
 */
static void bt_ready(int err)
{
    if (err) {
        printk("bt_enable init failed with err %d\n", err);
        return;
    }
    printk("Bluetooth initialized OK\n");

	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
        if (err) {
            printk("hwinfo_get_device_id failed with err %d\n", err);
        }
	}

    printk("\n%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n\n",
            dev_uuid[15], dev_uuid[14], dev_uuid[13], dev_uuid[12],dev_uuid[11], dev_uuid[10], dev_uuid[9], dev_uuid[8],
            dev_uuid[7], dev_uuid[6], dev_uuid[5], dev_uuid[4],dev_uuid[3], dev_uuid[2], dev_uuid[1], dev_uuid[0]);


    err = bt_mesh_init(&prov, &comp);

	if (err)
	{
		printk("bt_mesh_init failed with err %d\n", err);
		return;
	}

	printk("Mesh initialised OK: first element address: 0x%04x\n",elements[0].addr);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	    printk("Settings loaded\n");
	}

    if (!bt_mesh_is_provisioned()) {
    	printk("Node has not been provisioned - beaconing\n");
		bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
        prov_led(false);
	} else {
    	printk("Node has already been provisioned\n");
	    printk("Node unicast address: 0x%04x\n",elements[0].addr);
        prov_led(true);
	}

}

void main(void)
{
    int err;
    printk("\nBluetooth Mesh Atmospheric Sensor Node\n\n");

    printk("calling init_leds\n");
    init_leds();

    printk("calling configure_bme280_sensor\n");
    configure_bme280_sensor();

    printk("calling button_init\n");
    err = button_init(mesh_reset_work);
    if (err)
    {
        printk("button_init failed with err %d\n", err);
        return;
    }

    printk("calling bt_enable\n");
    err = bt_enable(bt_ready);
    if (err)
    {
        printk("bt_enable failed with err %d\n", err);
        return;
    }
}
