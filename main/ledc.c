/*
 * generate 4.4336MHz clock for TCM3105
 */
#ifdef CONFIG_ESP_LEDC
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

//#define TCM3105_CLK_PIN		GPIO_NUM_15
#define TCM3105_CLK_PIN		CONFIG_ESP_LEDC_PIN
#define TCM3105_CLK_FREQ	4433600 // TCM3105 clock frequency
//#define TCM3105_CLK_FREQ	4444444 // 40MHz / 9, no fractional
//#define TCM3105_CLK_FREQ	4454400 // 19200 * 232
//#define TCM3105_CLK_FREQ	6553600 // for 2400baud
//#define TCM3105_CLK_FREQ	4000000 // for 1200baud?
#define LEDC_TIMER_1_BIT	1

#define TCM3105_TRS_PIN		GPIO_NUM_12
#define TCM3105_TRS_FREQ	19110 // 1200baud by 16 times?
//#define TCM3105_TRS_FREQ	19200 // 1200baud by 16 times?

// 4.4336MHz clock generate for TCM3105
void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = TCM3105_CLK_FREQ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer)); // 4.4336MHz

#if 0
    ledc_timer.freq_hz = TCM3105_TRS_FREQ;
    ledc_timer.timer_num = LEDC_TIMER_1;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer)); // 19.11kHz
#endif

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 1,
        .gpio_num   = TCM3105_CLK_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel)); // clock for 3105

#if 0
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = TCM3105_TRS_PIN;
    ledc_channel.timer_sel = LEDC_TIMER_1;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel)); // CLK for TRS
#endif
}
#endif
