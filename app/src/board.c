#include <zephyr/kernel.h>

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "board.h"

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
static void led_0_timer_handler(struct k_timer *timer_id)
{
    gpio_pin_toggle_dt(&led[0]);
}

static void led_0_stop_handler(struct k_timer *timer_id)
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
void init_leds()
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
 *  Buttons
 *  ================================================================
 */ 
static struct k_work sw0_work;


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
int button_init(k_work_handler_t sw0_handler)
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

    k_work_init(&sw0_work, sw0_handler);

    gpio_init_callback(&gpio_cb, sw0_cb, BIT(sw0.pin));
    gpio_add_callback(sw0.port, &gpio_cb);

    return 0;
}

