#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/settings/settings.h>
#include <zephyr/drivers/hwinfo.h>

#include "mesh.h"
#include "board.h"



// states and state changes
static uint8_t dev_uuid[16] = { 0xef, 0xbe, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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
