#ifndef _BOARD_H
#define _BOARD_H

#include <zephyr/kernel.h>

void init_leds(void);
int button_init(k_work_handler_t sw0_handler);
void attn_led(bool state);
void prov_led(bool state);

#endif
