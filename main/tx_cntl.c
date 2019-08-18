/*
 * tx_cntl.c
 *
 * control radio transmitter
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <freertos/Freertos.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "tx_cntl.h"
#include "gpio.h"

#ifdef CONFIG_KISS_FULLDUPLEX
#define FULLDUPLEX	CONFIG_KISS_FULLDUPLEX
#else
#define FULLDUPLEX	1
#endif

#ifdef CONFIG_KISS_TXDELAY
#define TXDELAY		CONFIG_KISS_TXDELAY
#else
#define TXDELAY		50
#endif

#ifdef CONFIG_KISS_P
#define DEFAULT_P	CONFIG_KISS_P
#else
#define DEFAULT_P	63
#endif

#ifdef CONFIG_KISS_SLOTTIME
#define SLOTTIME	CONFIG_KISS_SLOTTIME
#else
#define SLOTTIME	10
#endif

#ifdef CONFIG_TNC_PROTOCOL
#define TNC_DEFAULT_MODE	CONFIG_TNC_PROTOCOL
#else
#define TNC_DEFAULT_MODE	FX25_PARITY_16
#endif

static int tnc_mode = CONFIG_TNC_PROTOCOL;

// KISS timing parameter resolution
#define TIME_UNIT	10	// 10 milli sec

// check CDT interval if CDT is ON
#define WAIT_CDT_TIME (10 / portTICK_PERIOD_MS)

#define TAG "TX_CNTL"

static unsigned int tx_seed = 1;

static uint8_t fullduplex = !FULLDUPLEX;
static int txdelay = (TXDELAY * TIME_UNIT) / portTICK_PERIOD_MS;
static int parameter_p = (DEFAULT_P + 1) * (RAND_MAX / 256 + 1) - 1;
static int slottime = (SLOTTIME * TIME_UNIT) / portTICK_PERIOD_MS;

void wait_carrier_inactive(void)
{
    while (!fullduplex) {
        if (!gpio_get_level(GPIO_CDT_PIN)) return;
	vTaskDelay(WAIT_CDT_TIME);
    }
}

void wait_txdelay(void)
{
    vTaskDelay(txdelay);
}

int p_persistent(void)
{
    return fullduplex || (rand_r(&tx_seed) <= parameter_p);
}

void wait_slottime(void)
{
    if (fullduplex) return;

    vTaskDelay(slottime);
}

void set_fullduplex(int ch, int duplex)
{
    if (ch != 0) return;

    fullduplex = duplex;
    ESP_LOGI(TAG, "fullduplex: %d", fullduplex);
}

void set_txdelay(int ch, int delay)
{
    if (ch != 0) return;

    txdelay = (delay * TIME_UNIT) / portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "txdelay: %d", txdelay);
}

void set_parameter_p(int ch, int p)
{
    if (ch != 0) return;

    parameter_p = (p + 1) * (RAND_MAX / 256 + 1) - 1;
    ESP_LOGI(TAG, "P: %d, parameter_p: %d", p, parameter_p);
}

void set_slottime(int ch, int delay)
{
    if (ch != 0) return;

    slottime = (delay * TIME_UNIT) / portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "slottime: %d", slottime);
}

void set_sethardware(int ch, uint8_t buf[], int size)
{
    uint8_t cmd;
    uint8_t param;

    if (ch != 0) return;
    if (size < 1) return;

    cmd = buf[0];
    switch (cmd) {
	case FX25_SETMODE:
	    if (size >= 2) {
		param = buf[1];
	    	switch (param) {
		case  0:
		   tnc_mode = AX25_MODE;	// AX.25 mode
		   break;
		case 16:
		   tnc_mode = FX25_PARITY_16;// FX.25 mode, parity 16
		   break;
		case 32:
		   tnc_mode = FX25_PARITY_32;// FX.25 mode, parity 32
		   break;
		case 64:
		   tnc_mode = FX25_PARITY_64;// FX.25 mode, parity 64
		   break;
		}
	    }
    }

    ESP_LOGI(TAG, "sethardware: ch: %x, size: %d", ch, size);
}

int get_tnc_mode(void)
{
    return tnc_mode;
}
