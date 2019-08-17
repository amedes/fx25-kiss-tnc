/*
 * FX.25 KISS TNC config parameter
 */
#ifndef __CONFIG_H__
#define __CONFIG_H__ 1

#include "sdkconfig.h"

// KISS default parameter
#define TXDELAY CONFIG_KISS_TXDELAY
#define P	CONFIG_KISS_P
#define SLOTTIME CONFIG_KISS_SLOTTIME
#define FULLDUPLEX CONFIG_KISS_FULLDUPLEX

// WiFi module ESP32 GPIO config
#define GPIO_RXD_PIN CONFIG_ESP_GPIO_RXD
#define GPIO_CDT_PIN CONFIG_ESP_GPIO_CDT
#define GPIO_TXD_PIN CONFIG_ESP_GPIO_TXD
#define GPIO_PTT_PIN CONFIG_ESP_GPIO_PTT
#define GPIO_LED_PIN CONFIG_ESP_GPIO_LED

#endif /* __CONFIG_H__ */
