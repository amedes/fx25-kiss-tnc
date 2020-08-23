/*
 * FX.25 KISS TNC for ESP-WROOM-32 WiFi module
 *
 *   Bell 202 modem chip: TCM3105
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>
#include <freertos/freertos.h>
#include <freertos/task.h>

#include <esp_clk.h>
#include <esp_sleep.h>
#include <driver/uart.h>
#include <esp_log.h>

#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "config.h"
#include "rmt.h"
#include "gpio.h"
#include "mcpwm.h"
#include "fx25_encode.h"
#include "fx25_decode.h"
#include "rs.h"
#include "ax25.h"
#include "fx25tag.h"
#include "bit_stuffing.h"
#include "ax25_decode.h"
#include "uart.h"
#include "wifi.h"
#include "tx_cntl.h"

#ifdef CONFIG_ESP_DAC
#include "dac.h"
#endif

#ifdef CONFIG_ESP_LEDC
#include "ledc.h"
#endif

#include "packet_table.h"

//#define USE_NOTIFICATION 1 // use xTaskNotification() for TX and RX synchronization
//#define USE_ACK 1
#define ACK_QUEUE_LEN 1
#define TAG "main"

#define CHECK_MISS 1	// check error correction miss

//#define FX25_DATA_SIZE (4 + 8 + 255 + 1) // pre + tag + rs + post
#define PARITY_SYMBOLS 16 // len of parity

#define BIT_TIME ((80000000 + BAUD/2) / BAUD) // APB clock 80MHz, 1200bps
#define BIT_TIME2 (BIT_TIME / 2)

#define FX25_BUF_SIZE (4 + 8 + 8 + 255*15 + 1) // maximum FX.25 packet size, first byte is type indicator
#define AX25_BUF_SIZE (239*15 - 2)
#define AX25_NRZI_SIZE (((AX25_BUF_SIZE * 8 * 6 + 4) / 5 + 7) / 8)

#define RXD_QUEUE_LEN 1024

#define BIT_LEN_MAX 32 // threthold for packet termination

// KISS mode asynchronouse frame format
#define FEND 0xc0	// frame end
#define FESC 0xdb	// frame escape
#define TFEND 0xdc	// transposed frame end
#define TFESC 0xdd	// transposed trame escape

#define UART_NUM UART_NUM_0
#define PRINT_ESC 1	// escape non-printable character for debug

//static int tx_pkts = 0;
//static int ax25_pkts = 0;
//static int ax25_nrzi_pkts = 0;
static int ax25_nrzi_ec_try = 0;
static int ax25_nrzi_ec_ok = 0;
static int ax25_nrzi_ec_miss = 0;
//static int fx25_pkts = 0;
static int fx25_rs_try = 0;
static int fx25_rs_ok = 0;
static int fx25_rs_miss = 0;
static int ax25_nrzi_bits = 0;
static int fx25_bits = 0;

#ifdef CHECK_MISS
static uint8_t *test_packet;
static int test_packet_len;
static uint8_t *test_fx25_buf;
static int test_fx25_len;
#endif

#ifdef CONFIG_TNC_BEACON
static TaskHandle_t tx_task_handle;
#endif

#ifdef USE_ACK
static QueueHandle_t ack_queue;
#endif

/*
 * AX.25 receive processing
 *
 * rxd: last edge time
 * rxd0: previous edge time
 */
static uint64_t ax25_bit_sum[2] = { 0, 0 };
static uint64_t ax25_bit_cnt[2] = { 0, 0 };

static uint64_t fx25_bit_sum[2] = { 0, 0 };
static uint64_t fx25_bit_cnt[2] = { 0, 0 };

//static const int bit_time[3] = { 66666, 66667, 66667 };

static int ax25_rx(uint32_t rxd, uint32_t rxd0)
{
    static int bit_tm = 0;
    static int bit_sum = 0;
    uint32_t t = rxd - rxd0;
    int level = rxd & 1; // 0: positive edge, 1: negative edge
    int pkt_ack = 0;
    int decode_ok = 0;
    //static int bit_time_cnt = 0;

    if (t > BIT_LEN_MAX * BIT_TIME) {

		// bit sync and reset
		bit_tm = 0;
		bit_sum = 0;
		ax25_nrzi_bit(-1, NULL, 0);

#ifdef CONFIG_TNC_STATISTICS
		fx25_stat.ax25_ok = 0; // clear AX.25 decode success flag
#endif

		return 0;
    }

    while (bit_tm + t >= BIT_TIME) {
    //while (bit_tm + t >= bit_time[bit_time_cnt]) {
		static uint8_t ax25_nrzi_buf[AX25_NRZI_SIZE];
		uint32_t addt = BIT_TIME - bit_tm;
		//uint32_t addt = bit_time[bit_time_cnt] - bit_tm;
		//bit_time_cnt = (bit_time_cnt + 1) % 3;

		if (level) bit_sum += addt;
		t -= addt;

		int lv = bit_sum >= BIT_TIME2;

		// statistics
		ax25_bit_sum[lv] += bit_sum;
		ax25_bit_cnt[lv]++;

		bit_sum = 0;
		bit_tm = 0;

		int nrzi_len = ax25_nrzi_bit(lv, ax25_nrzi_buf, AX25_NRZI_SIZE);

		if (nrzi_len == -2) { // flag found

			//printf("ax25_rx: bit sync, t = %u\n", t);
			// bit sync
			t = ((t + BIT_TIME2) / BIT_TIME) * BIT_TIME;
		}

		if (nrzi_len <= 0) continue;

//#define AX25_MIN_LEN (7 * 2 + 1 + 1 + 2) // address * 2 + Control + Info + FCS
#define AX25_MIN_LEN 1

		if (nrzi_len >= AX25_MIN_LEN * 8) {
			static uint8_t ax25_buf[AX25_BUF_SIZE];
			int bit_len = nrzi_decode(ax25_buf, AX25_BUF_SIZE, ax25_nrzi_buf, nrzi_len);

#if 0
			if (bit_len < 0) {
				printf("ax25_rx: ax25_nrzi_buf, nrzi_len = %d\n", nrzi_len);
				print_buf(ax25_nrzi_buf, (nrzi_len + 7) / 8);
			}
#endif

			if ((bit_len >= AX25_MIN_LEN * 8) && (bit_len % 8 == 0)) {
				int ax25_len = bit_len / 8;

				if (ax25_fcs_check(ax25_buf, ax25_len)) {
					// success decoding AX.25 packet

					pkt_ack++;
					ax25_nrzi_bits++;
					decode_ok++;
				}
			}
		}
    }
    if (level) bit_sum += t;
    bit_tm += t;

    return pkt_ack;
}

/*
 * FX25 receive processing
 */
static int fx25_rx(uint32_t rxd, uint32_t rxd0, buffer_info *fx_buff, code_info *fx_info)
{
    static int bit_tm = 0;
    static int bit_sum = 0;
    uint32_t t = rxd - rxd0;
    int level = rxd & 1; // 0: positive edge, 1: negative edge
    int pkt_ack = 0;
    //static int bit_time_cnt = 0;

    if (t > BIT_LEN_MAX * BIT_TIME) {

		//printf("fx25_rx: bit sync, bit_tm = %u, bit_sum = %u\n", bit_tm, bit_sum);

		// bit sync
		bit_tm = 0;
		bit_sum = 0;
		clear_code_info(fx_info);
		
		return 0;
    }

    while (bit_tm + t >= BIT_TIME) {
    //while (bit_tm + t >= bit_time[bit_time_cnt]) {
		static uint8_t fx25_buf[FX25_BUF_SIZE];
		uint32_t addt = BIT_TIME - bit_tm;
		//uint32_t addt = bit_time[bit_time_cnt] - bit_tm;
			//bit_time_cnt = (bit_time_cnt + 1) % 3;

		if (level) bit_sum += addt;
		t -= addt;

		int lv = bit_sum >= BIT_TIME2;

		// statistics
		fx25_bit_sum[lv] += bit_sum;
		fx25_bit_cnt[lv]++;

			//if ((bit_sum > BIT_TIME2/2) && (bit_sum < BIT_TIME2*3/2)) printf("fx25_rx(): bit_tm = %u, bit_sum = %u\n", bit_tm, bit_sum);

		bit_sum = 0;
		bit_tm = 0;

		int tag_no = fx25_decode_bit(lv, fx25_buf, FX25_BUF_SIZE, fx_info); 

		if (tag_no == -2) { // flag found

			//printf("fx25_rx: bit sync, t = %u\n", t);

			// bit sync
			t = ((t + BIT_TIME2) / BIT_TIME) * BIT_TIME;
		}

		if (tag_no > 0) { // correlation tag is detected
			static uint8_t ax25_buf[AX25_BUF_SIZE];
			int rs_code_size = tags[tag_no].rs_code;
			int ax25_len = bitstuff_decode(ax25_buf, AX25_BUF_SIZE, &fx25_buf[1], rs_code_size - 1); // buf[0] is AX.25 flag (7E)


			if ((ax25_len > 2) && ax25_fcs_check(ax25_buf, ax25_len)) {

				pkt_ack++;
				fx25_bits++;

			} else {


				fx25_rs_try++;

				static uint8_t err_buf[FX25_BUF_SIZE];
				memcpy(err_buf, fx25_buf, rs_code_size);

				int rs_result = fx25_rsdecode(fx25_buf, tag_no); // RS error correction


				if (rs_result >= 0) {

					ESP_LOGD(TAG, "fx25_rx: RS error correction: %d symbols", rs_result);
					//print_diff(err_buf, fx25_buf, rs_code_size);
					
					ax25_len = bitstuff_decode(ax25_buf, AX25_BUF_SIZE, &fx25_buf[1], rs_code_size - 1);

					if ((ax25_len > 2) && ax25_fcs_check(ax25_buf, ax25_len)) {

						fx25_bits++;
						fx25_rs_ok++;
						pkt_ack++;


						ESP_LOGD(TAG, "fx25_rx: FCS error");
					}

				}
			}
		}
    }
    if (level) bit_sum += t;
    bit_tm += t;

    return pkt_ack;
}

#include "fx25_code_info.h"

/*
 * TNC receive task
 */
void rx_task(void *p)
{
    QueueHandle_t capqueue = *(QueueHandle_t *)p;
    uint32_t rxd0 = 0, /*rxd1 = 0,*/ rxd;    
    int rc;
    //static uint8_t ax25_buf[AX25_BUF_SIZE];
    //static uint8_t ax25_nrzi_buf[AX25_NRZI_SIZE];
    //static uint8_t ax25_buf2[AX25_BUF_SIZE];
    //static uint8_t fx25_buf[FX25_BUF_SIZE];
    //int ax25_cnt = 0;
    //int ec_cnt = 0;
    //int len;
    //int nrzi_cnt = 0;
    //int st = 0;
    //int bit_sum = 0;
    //int bit_tm = 0;
    int pkt_ack = 0;

	code_info fx25_info;
	buffer_info buff_info;

    while (1) {
		rxd0 = rxd;
		rc = xQueueReceive(capqueue, &rxd, 1000); // wait 1000 ticks
		if (rc != pdTRUE) continue;

		if (cap_queue_err) {
			printf("mcpwm: capture error: %d\n", cap_queue_err);
			cap_queue_err = 0;
		}

		// check RXD edge polarity
		if (((rxd0 ^ rxd) & 1) == 0) {
			// may be lost interrupt
			uint32_t tmp = (rxd - rxd0) >> 3;
			ESP_LOGI(TAG, "wrong RXD edge polarity: time diff %u.%u us", tmp / 10, tmp % 10);
		}


		// AX.25 packet decode
		pkt_ack = ax25_rx(rxd, rxd0);

		//uint32_t t = rxd - rxd0;
		//int level = rxd & 1; // positive edge 0, negative edge 1
		//int bits = (t + BIT_TIME2) / BIT_TIME; // number of bit length between edges

		// decode FX.25 packet
		pkt_ack += fx25_rx(rxd, rxd0, &buff_info, &fx25_info);

    }
}

#define PKT_LEN 1024
#define BITS_BUF_LEN 1195

//static uint8_t pkt[256];

void app_main()
{
    static QueueHandle_t capqueue;
    BaseType_t rc;
    TaskHandle_t xHandle;

    // set log level
#ifdef CONFIG_TNC_LOG_LEVEL
    esp_log_level_set("*", CONFIG_TNC_LOG_LEVEL);
#else
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

#ifdef CONFIG_RS_DIREWOLF_GP
    ESP_LOGI(TAG, "RS code Direwolf compatible mode");
#else
    ESP_LOGI(TAG, "RS code old FX.25 KISS TNC compatible mode");
#endif

    capqueue = xQueueCreate(RXD_QUEUE_LEN, sizeof(uint32_t));

    mcpwm_initialize(capqueue); // capture RXD state
    gpio_init(); // LED reflect GPIO state (connect CDT or RXD)
    rmt_tx_init(); // send pulse train to modem (TXD)
 
#ifdef CONFIG_ESP_DAC
    dac_init();	// RXB and CDL
#endif

#ifdef CONFIG_ESP_LEDC
    ledc_init(); // clock for TCM3105
#endif

#ifdef USE_WIFI
    wifi_start();	// start WiFi
#endif

    uart_init(); // set TX and RX buffer, invoke service task

    if (rs_init() != RS_OK) {; // init Reed Solomon routine
		printf("rs_init() error\n");
		return;
    }

    fx25tag_init(); // generate FX.25 PN tags 

    // receiver task
    rc = xTaskCreatePinnedToCore(rx_task, "rx_task", 1024*4, &capqueue, tskIDLE_PRIORITY+1, &xHandle, tskNO_AFFINITY);
    if (rc != pdPASS) {
		return;
    }

#ifdef CONFIG_TNC_BEACON 
    // transmitter task
    rc = xTaskCreatePinnedToCore(tx_task, "tx_task", 1024*4, NULL, tskIDLE_PRIORITY+1, &tx_task_handle, tskNO_AFFINITY);
    if (rc != pdPASS) {
		return;
    }
#endif

#ifdef CONFIG_TNC_DEMO_MODE
    printf("***Warning: FX.25 KISS TNC enter demonstration mode***\n");
#endif
}
