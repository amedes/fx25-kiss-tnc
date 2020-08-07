/*
 * rmt.c
 *
 * send FX.25 packet to modem
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/rmt.h"
#include "esp_system.h"
#include "esp_log.h"

#ifdef __has_include
#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif
#endif

#include "config.h"
#include "rmt.h"
#include "gpio.h"
#include "tx_cntl.h"

//#define BAUD 1200

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO	GPIO_TXD_PIN	// TXD to TCM3105
//#define RMT_TX_GPIO 14

#define RMT_RX_CHANNEL RMT_CHANNEL_1
#define RMT_RX_GPIO 14

//#define RMT_CLK_DIV (80)
//#define RMT_CLK_DIV (3)
#define RMT_DURATION ((80*1000*1000 / RMT_CLK_DIV + BAUD/2) / BAUD) // 1200bps

#define RMT_TAG "RMT"

#ifdef CONFIG_TNC_DEMO_MODE
#define AN_BIT 0
#define AN_BURST 1

static uint32_t an_bit = UINT32_MAX / 1000; // 1e-3
static uint32_t an_burst = UINT32_MAX / 2; // 0.5
static int an_state = AN_BIT; // bit error state

/*
 * add bit error for demo
 */
static int noise(int level)
{
    switch (an_state) {
		case AN_BIT:
			if (esp_random() < an_bit) {
				an_state = AN_BURST;
				return !level; // invert
			}
			break;
		case AN_BURST:
			if (esp_random() < an_burst) {
				return esp_random() & 1; // random bit
			}
			an_state = AN_BIT;
			break;
    }

    return level; // no error
}
#endif

static int gcd(int x, int y)
{
    int a, b, r;

    if (x <= 0 || y <= 0) return -1;

    if (x < y) {
		a = x;
		b = y;
    } else {
		a = y;
		b = x;
    }

    // Euclidean mutual division method
    while (1) {
		r = b % a;

		if (r == 0) break;

		b = a;
		a = r;
    }

    return a;
}

static int rmt_div;
static int rmt_bit_apb_clocks;

#define APB_CLK		(80 * 1000 * 1000)
#define RMT_DURATION_MAX 32767
#define RMT_CLK_DIV	((APB_CLK + (BAUD * RMT_DURATION_MAX - 1)) / (BAUD * RMT_DURATION_MAX))

static void copy_to_rmt_init(void)
{
    int d = gcd(APB_CLK, BAUD * RMT_CLK_DIV);

    if (d <= 0) {
		ESP_LOGE(RMT_TAG, "gcd(%d, %d) = %d", APB_CLK, BAUD * RMT_CLK_DIV, d);
		abort();
    }

    rmt_bit_apb_clocks = APB_CLK / d;
    rmt_div = BAUD * RMT_CLK_DIV / d;

    ESP_LOGI(RMT_TAG, "rmt_bit_apb_clocks: %d, rmt_div: %d", rmt_bit_apb_clocks, rmt_div);
}

/*
 * copy packet data to rmt hardware with NRZI conversion
 */
static void IRAM_ATTR copy_to_rmt(const void *src, rmt_item32_t *dest, size_t src_size,
		size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    static int level = 1; // save level
    static int rem = RMT_CLK_DIV / 2; // DDA remainder

    //ESP_LOGI(RMT_TAG, "src_size = %d, wanted_num = %d", src_size, wanted_num);

    if (!src || !dest) {
		*translated_size = 0;
		*item_num = 0;

		return;
    }

    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item16_t *pdest = (rmt_item16_t *)dest;

    while ((size < src_size) && (num < wanted_num)) {
		for (int m = *psrc++ | 0x100; m >= 2; m >>= 1) { // 0x100 is sentinel

	    // calculate one bit duration used by DDA method
			pdest->duration = (rmt_bit_apb_clocks - rem + (rmt_div - 1)) / rmt_div;
			rem += pdest->duration * rmt_div - rmt_bit_apb_clocks;

			if ((m & 1) == 0) level = !level; // NRZI
#ifdef CONFIG_TNC_DEMO_MODE
			pdest->level = noise(level);
#else
			pdest->level = level;
#endif
			pdest++;
		}
		num += 4;
		size++;
    }

    if (size >= src_size) { // all src processed
		level = 1; // reset level
		rem = RMT_CLK_DIV / 2; // reset remainder
    }

    *translated_size = size;
    *item_num = num;

    //ESP_LOGI(RMT_TAG, "translated_size = %d, item_num = %d", size, (num16 + 1) / 2);
}

#if 0
static void make_pulse(rmt_item16_t *item16, int du, int lv)
{
#if 1
    static int du_tab[] = { 834, 833, 833, 0 }; // 80MHz / CLK_DIV / BAUD = 2500
    static int idx = 0;
    int sum = 0;
    
    while (--du >= 0) {
    	sum += du_tab[idx++];
		idx %= 3;
    }
    item16->duration = sum;
#else
    item16->duration = du * RMT_DURATION;
#endif
    item16->level = lv;
}

/* make pulses from byte */
static int byte_to_pulse(rmt_item16_t *item16, int size, uint8_t c, int *dup, int *lvp, int bsflag)
{
    int i = 0;
    uint8_t b;
    int du = *dup;
    int lv = *lvp; 

    bsflag = false; // force no bit stuffing

    for (b = 1; b != 0; b <<= 1) { // LSB first
		if (b & c) { // send 1
			du++;
			if (bsflag && (du >= 6)) { // bit stuffing
				if (i >= size) break;
				make_pulse(&item16[i++], du, lv);
				du = 1;
				lv = !lv;
			}
		} else { // send 0
			if (i >= size) break;
			make_pulse(&item16[i++], du, lv);
			du = 1;
			lv = !lv;
		}
    }
    *dup = du;
    *lvp = lv;

    return i;
}
#endif

#define ITEM32_SIZE (2048*8/2)		// 32bit holds 2 pulse info
#define ITEM16_SIZE (ITEM32_SIZE*2)	// 16bit holds 1 pulse info

#define RMT_BUF_LEN 2048		// temporary buffer

/*
 * convert to NRZI and send to RMT H/W
 */
static void rmt_send_split(uint8_t *item[], size_t size[])
{
    static uint8_t buf[RMT_BUF_LEN];
    int i;
    int len = 0;

    /* copy packet to buffer */
    for (i = 0; i < 2; i++) {

		if (item[i] == NULL) break;
		if (len + size[i] > RMT_BUF_LEN) return; // packet too large

		memcpy(&buf[len], item[i], size[i]);
		len += size[i];
    }

    if (len <= 0) return;

    // send packet to air
    ESP_ERROR_CHECK(rmt_write_sample(RMT_TX_CHANNEL, buf, len, true)); // wait for transmission
}

static RingbufHandle_t rmt_ringbuf;

static void rmt_send_task(void *p)
{
    RingbufHandle_t rb = (RingbufHandle_t)p;
    uint8_t *item[2];
    size_t size[2];

    while (1) {
	// wait incoming packet
		if (xRingbufferReceiveSplit(rb, (void **)&item[0], (void **)&item[1], &size[0], &size[1], portMAX_DELAY) == pdTRUE) {

			// process half duplex
			while (1) {
				wait_carrier_inactive();	// wait for channel clear
				if (p_persistent()) break;	// do p-persistent
				wait_slottime();
			}

			// begin transmission
			ptt_control(PTT_ON);
			wait_txdelay();

			// send all queued frames
			do {

				// send frame
				rmt_send_split(item, size);
				vRingbufferReturnItem(rb, item[0]);
				if (item[1]) vRingbufferReturnItem(rb, item[1]);

				// check queue
				UBaseType_t items;
#ifdef ESP_IDF_VERSION
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
				vRingbufferGetInfo(rb, NULL, NULL, NULL, NULL, &items);
#else
				vRingbufferGetInfo(rb, NULL, NULL, NULL, &items);
#endif
#else
				vRingbufferGetInfo(rb, NULL, NULL, NULL, &items);
#endif
				if (items <= 0) break; // exit if no queued frame

				// read next frame
			} while (xRingbufferReceiveSplit(rb, (void **)&item[0], (void **)&item[1], &size[0], &size[1], 0) == pdTRUE); 
			// no wait

	    	// end transmittion
	    	ptt_control(PTT_OFF);
		}
    }
}

/*
 * send async KISS frame data
 *
 * buf: first byte contain channel
 */
void send_packet(uint8_t buf[], int size, int wait)
{
    if ((buf == NULL) || (size <= 0)) return;

    if (xRingbufferSend(rmt_ringbuf, buf, size, wait ? portMAX_DELAY : 0) != pdTRUE) {; // wait for sending if wait is true
		ESP_LOGD(RMT_TAG, "ring buffer full: discard %d byte", size);
    }
}

#define RMT_RINGBUF_SIZE (1024*32)

/*
 * Initialize the RMT Tx channel
 */
void rmt_tx_init(void)
{
    rmt_config_t config = {
    	.rmt_mode = RMT_MODE_TX,
    	.channel = RMT_TX_CHANNEL,
    	.gpio_num = RMT_TX_GPIO,
    	.mem_block_num = 1,
    	.tx_config.loop_en = 0,
#if 1
    	.tx_config.carrier_en = 0,
    	.tx_config.idle_output_en = 1,
    	.tx_config.idle_level = 1,
#else
	// IR setting
		.tx_config.carrier_freq_hz = 37900, // 38kHz
		.tx_config.carrier_duty_percent = 33, // duty 1:3
		.tx_config.carrier_level = 1,
    	.tx_config.carrier_en = 1,
    	.tx_config.idle_output_en = 1,
    	.tx_config.idle_level = 0,
#endif
    	.clk_div = RMT_CLK_DIV
    };
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    copy_to_rmt_init(); // initialize variables used in copy_to_rmt()
    ESP_ERROR_CHECK(rmt_translator_init(config.channel, copy_to_rmt));

    rmt_ringbuf = xRingbufferCreate(RMT_RINGBUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
    if (rmt_ringbuf == NULL) {
		ESP_LOGI(RMT_TAG, "xRingbufferCreate() fail");
		abort();
    }

#ifdef CONFIG_TNC_DEMO_MODE
    float x;
    //struct timeval tv;

    x = atof(CONFIG_TNC_BIT_ERR_RATE);
    printf("Bit error rate %0.3g\n", x);
    if ((x > 0.0) && (x < 1.0)) an_bit = x * UINT32_MAX;
    x = atof(CONFIG_TNC_BURST_ERR_RATE);
    printf("Burst error rate %0.3g\n", x);
    if ((x > 0.0) && (x < 1.0)) an_burst = x * UINT32_MAX;
    printf("Bit error rate: %.3g, Burst error rate: %.3g\n", (float)an_bit / UINT32_MAX, (float)an_burst / UINT32_MAX);

    //gettimeofday(&tv, NULL);
    //an_seed = tv.tv_usec;
#endif

    configASSERT(xTaskCreatePinnedToCore(rmt_send_task, "rmt_send_task", 2048, rmt_ringbuf, tskIDLE_PRIORITY+1, NULL, tskNO_AFFINITY) == pdPASS);
}
