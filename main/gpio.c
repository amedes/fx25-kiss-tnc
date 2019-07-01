/*
 * GPIO setting
 */
#include "driver/gpio.h"

#include "config.h"
#include "gpio.h"

//#define GPIO_RXD_PIN 39		// input RXD signal
//#define GPIO_CDT_PIN 36		// input CDT signal
//#define GPIO_LED_PIN 2		// output CDT state to LED
//#define GPIO_PTT_PIN 15		// output PTT control

#define ESP_INTR_FLAG_DEFAULT 0

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    gpio_set_level(GPIO_LED_PIN, gpio_get_level(gpio_num));
#if 0
    pcnt_get_counter_value(PCNT_CHANNEL_0, &count);
    pcnt_counter_clear(PCNT_CHANNEL_0);
#endif
}

void ptt_control(int onoff)
{
    gpio_set_level(GPIO_PTT_PIN, onoff);
    //gpio_set_level(GPIO_PTT_PIN, !onoff); // open drain output
}

void gpio_init(void)
{
    // RXD input pin initialization
    gpio_config_t io_conf = {
	.intr_type = GPIO_INTR_ANYEDGE,
	.mode = GPIO_MODE_INPUT,
	.pin_bit_mask = (1ULL << GPIO_RXD_PIN),
	.pull_down_en = 0,
	.pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //gpio_isr_handler_add(GPIO_RXD_PIN, gpio_isr_handler, (void *)GPIO_RXD_PIN);
    //gpio_intr_enable(GPIO_RXD_PIN);

    // CDT input pin initialization
    io_conf.pin_bit_mask = 1ULL << GPIO_CDT_PIN;
    gpio_config(&io_conf);
    gpio_isr_handler_add(GPIO_CDT_PIN, gpio_isr_handler, (void *)GPIO_CDT_PIN);
    gpio_intr_enable(GPIO_CDT_PIN);

    // LED output pin initialization
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_LED_PIN);
    gpio_config(&io_conf);
    gpio_set_level(GPIO_LED_PIN, gpio_get_level(GPIO_CDT_PIN));

    // PTT output pin initialization
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_PTT_PIN);
    gpio_config(&io_conf);
    ptt_control(0); // PTT OFF
}
