#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

#define LED_PIN_1 GPIO_NUM_26
#define LED_PIN_2 GPIO_NUM_27

int init_leds();
void swap_leds();
void update_leds();

extern int which_led;
extern int active;

#endif
