/*
 * gpio.h
 *
 * GPIO PIN definition
 */
#ifndef GPIO_TXD_PIN
#define GPIO_TXD_PIN 12		// output TXD signal
#define GPIO_RXD_PIN 39		// input RXD signal
#define GPIO_CDT_PIN 36		// input CDT signal
#define GPIO_LED_PIN 2		// output CDT state to LED
#define GPIO_PTT_PIN 15		// output PTT control
#endif

void gpio_init(void);
void ptt_control(int onoff);
