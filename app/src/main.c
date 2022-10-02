#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/settings/settings.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>


// LEDs
#define ATTENTION_LED_SLEEP_MS 333

static const struct gpio_dt_spec led[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_3), gpios),
};

// Buttons
#define SW0 DT_ALIAS(sw0)

static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(SW0, gpios);


// states and state changes
static uint8_t dev_uuid[16] = { 0xef, 0xbe, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
*  LEDs
*  ================================================================
*
*  States:
*  User LED 1:
*   - blinking at 1.5 HZ: attention
*  User LED 2:
*   - on when provisioned, off otherwise
*/
// User LED 1 Timer config
void led_0_timer_handler(struct k_timer *timer_id)
{
    gpio_pin_toggle_dt(&led[0]);
}

void led_0_stop_handler(struct k_timer *timer_id)
{
    // Ensure LED is off
    gpio_pin_set_dt(&led[0], 0);
}

K_TIMER_DEFINE(led0_timer, led_0_timer_handler, led_0_stop_handler);

/**
 * attn_led(): set state of the attention LED
 * @state: enable or disable blinking
 *
 */
void attn_led(bool state)
{
    if (state) {
        k_timer_start(&led0_timer, K_NO_WAIT, K_MSEC(ATTENTION_LED_SLEEP_MS));
    } else {
        k_timer_stop(&led0_timer);
    }
}

/**
 * prov_led(): set state of provisioning led
 * @state: enable or disable LED
 *
 */
void prov_led(bool state)
{
    gpio_pin_set_dt(&led[1], state);
}

/**
 * init_leds(): Configure LED pins
 *
 */
static void init_leds()
{
    if (!(device_is_ready(led[0].port) && device_is_ready(led[1].port) && device_is_ready(led[2].port))) {
        printk("led device is not ready\n");
        return;
    }

    gpio_pin_configure_dt(&led[0], GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led[1], GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led[2], GPIO_OUTPUT_ACTIVE);

    gpio_pin_set_dt(&led[0], 0);
    gpio_pin_set_dt(&led[1], 0);
    gpio_pin_set_dt(&led[2], 0);
}

/*
   Buttons
   ================================================================
*/ 
struct k_work sw0_work;

/**
 * sw0_pressed(): reset mesh state and begin advertising
 * @work: ignored
 *
 */
static void sw0_pressed(struct k_work *work)
{
    bt_mesh_reset();
	printk("Mesh state is reset!\n");

	printk("Begin advertising!\n");
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

/**
 * sw0_cb(): user button 0 interrupt callback
 * 
 */
static void sw0_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    k_work_submit(&sw0_work);
}

/**
 * button_init(): configure user buttons
 *
 * Return: err
 */
static int button_init()
{
    int err;

    static struct gpio_callback gpio_cb;

    err = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
    if (err) {
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
    if (err) {
        return err;
    }

    k_work_init(&sw0_work, sw0_pressed);

    gpio_init_callback(&gpio_cb, sw0_cb, BIT(sw0.pin));
    gpio_add_callback(sw0.port, &gpio_cb);

    return 0;
}


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

// Health Server
// ========
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

#define CONFIG_SERVER_MODEL 0
#define HEALTH_SERVER_MODEL 1

// SIG models for the first element
static struct bt_mesh_model sig_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
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

    printk("calling button_init\n");
    err = button_init();
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
