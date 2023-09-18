#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "sensor.h"


const struct device *sensor;

struct bme280_measurements sensor_readings;

static const struct device *get_bme280_device(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme280);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");

		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);

	return dev;
}

void sample_sensor()
{
    int err;
    struct sensor_value temp, press, humidity;

    err = sensor_sample_fetch(sensor);
    if (err < 0) {
        printk("sensor_sample_fetch failed with err %d\n", err);
    }

    err = sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (err < 0) {
        printk("sensor_channel_get SENSOR_CHAN_AMBIENT_TEMP failed with err %d\n", err);
    }

    err = sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &press);
    if (err < 0) {
        printk("sensor_channel_get SENSOR_CHAN_PRESS failed with err %d\n", err);
    }

    err = sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, &humidity);
    if (err < 0) {
        printk("sensor_channel_get SENSOR_CHAN_HUMIDITY failed with err %d\n", err);
    }

    sensor_readings.temp = sensor_value_to_double(&temp);
    sensor_readings.press = sensor_value_to_double(&press);
    sensor_readings.humidity = sensor_value_to_double(&humidity);

    printk("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
            temp.val1, temp.val2, press.val1, press.val2,
            humidity.val1, humidity.val2);

}

void configure_bme280_sensor()
{
    sensor = get_bme280_device();
    if (sensor == NULL) {
        return;
    }

    printk("bme280 configured and ready\n");
}
