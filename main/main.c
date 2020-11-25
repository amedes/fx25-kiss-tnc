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
#include "fx25_code_info.h"

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
#define AX25_BUF_SIZE (4096)
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
	int level_prev = 0;
	int bit;
	int ax_pkt_error = 0;
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

		if (level < 0) { // reset variables
			//printf("fx25_get_frame(): level_prev = %d, bit_offset = %d\n", level_prev, bit_offset);
			level_prev = 1;
			clear_code_info(fx_info);
			return 0;
		}

		bit = !(level ^ level_prev); // decode NRZI
		level_prev = level;


		int sync_stat = fx25_get_frame(bit, fx_buff, fx_info); 

		
		// PN_SYNC_DONE || FRAME_ERROR || FREME_CONTINUE || FRAME_ENDED
		if (sync_stat == FRAME_ERROR) {

		}
		if (sync_stat == PN_SYNC_DONE) {
			ax_pkt_error = 0;
		}

		if (sync_stat == FRAME_ENDED) {
			static uint8_t ax25_buf[AX25_BUF_SIZE];
			int rs_code_size = tags[sync_stat].rs_code;
			int ax25_len = bitstuff_decode(ax25_buf, AX25_BUF_SIZE, &fx25_buf[1], rs_code_size - 1); // buf[0] is AX.25 flag (7E)


			if ((ax25_len > 2) && ax25_fcs_check(ax25_buf, ax25_len)) {

				pkt_ack++;
				fx25_bits++;

			} else {


				fx25_rs_try++;

				static uint8_t err_buf[FX25_BUF_SIZE];
				memcpy(err_buf, fx25_buf, rs_code_size);

				int rs_result = fx25_rsdecode(fx25_buf, sync_stat); // RS error correction


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
#define FX25 1
int rx_bit_receive(int bit, buffer_info *fx_buff, code_info *fx_info)
{
	int mode;
	int sync_stat = fx25_get_frame(bit, fx_buff, fx_info); 
	// PN_SYNC_DONE || FRAME_ERROR || FREME_CONTINUE || FRAME_ENDED || UNDEFINED_PN || BUFF_NOT_ENOUGH
	switch (sync_stat) {
		case UNDEFINED_PN :
			// Ignore for the time being.
		break;
		
		case FRAME_ERROR :
			// Ignore for the time being.
		break;
		
		case BUFF_NOT_ENOUGH :
			// このフレームはFX.25でのデコードをあきらめるしかない
			// 
		break;
		
		case PN_SYNC_DONE :
			// ここからFX.25フレーム
			// 並行してAX.25をデコードする場合も改めてここからなので
			// AX.25のデコードをリセット
			mode = FX25;
		break;
		
		case FRAME_CONTINUE :
		break;
		
		case FRAME_ENDED :
			int rs_code_size = tags[sync_stat].rs_code;
			int hoge = ax25_decode_bit(bit, fx_buff, AX25_BUF_SIZE);
			int ax25_len = bitstuff_decode(fx_buff, AX25_BUF_SIZE, fx_buff, rs_code_size - 1);


			if ((ax25_len > 2) && ax25_fcs_check(fx_buff, ax25_len)) {

//				pkt_ack++;
				fx25_bits++;

			} else {


				fx25_rs_try++;

				static uint8_t err_buf[FX25_BUF_SIZE];
				memcpy(err_buf, fx_buff, rs_code_size);

				int rs_result = fx25_rsdecode(fx_buff, sync_stat); // RS error correction


				if (rs_result >= 0) {

					ESP_LOGD(TAG, "fx25_rx: RS error correction: %d symbols", rs_result);
					//print_diff(err_buf, fx25_buf, rs_code_size);
					
					ax25_len = bitstuff_decode(fx_buff, AX25_BUF_SIZE, &fx_buff[1], rs_code_size - 1);

					if ((ax25_len > 2) && ax25_fcs_check(fx_buff, ax25_len)) {

						fx25_bits++;
						fx25_rs_ok++;



						ESP_LOGD(TAG, "fx25_rx: FCS error");
					}

				}
				packet_output(fx_buff->buff, ax25_len - 2);
			}
		break;
		
	}
	

}

void init_buffer_info(buffer_info *buff_info)
{
	buff_info->bit_pos = 0;
	buff_info->buff_size = AX25_BUF_SIZE;
}

#define NO_SYNC -1
#define SYNC_ERROR -2

void init_bitsync_info(bitsync_info *sync_info)
{
 	sync_info->bit_sum = 0;
	sync_info->bit_tm = 0;
 	sync_info->rxd0 = 0;
	sync_info->bit_level_prev = 1;
	sync_info->onbit = 0;

	return;
}

int rx_bit_sync(QueueHandle_t capqueue, bitsync_info *rxst_info)
{
	int rc;
	uint32_t rxd;
	uint32_t rxd0;
	uint32_t diff_time;
	int edge_polarity;

	if (rxst_info->onbit == 0) {
		rc = xQueueReceive(capqueue, &rxd, 1000); // wait 1000 ticks
		if (rc != pdTRUE) {
			return SYNC_ERROR;
		}
		if (cap_queue_err) {
			printf("mcpwm: capture error: %d\n", cap_queue_err);
			cap_queue_err = 0;
			return SYNC_ERROR;
		}

		rxd0 = rxst_info->rxd0;

		diff_time = rxd - rxd0;
		//static int bit_time_cnt = 0;

		// check RXD edge polarity
		if (((rxd0 ^ rxd) & 1) == 0) {
			// may be lost interrupt		
			ESP_LOGI(TAG, "wrong RXD edge polarity: time diff %u.%u us", diff_time>>3 / 10, diff_time>>3 % 10);
			return SYNC_ERROR;
		}
		rxst_info->rxd0 = rxd;

		if (diff_time > BIT_LEN_MAX * BIT_TIME) {
			// too long 1 bit
			ESP_LOGI(TAG, "wrong 1 bit length : time diff %u.%u us", diff_time>>3 / 10, diff_time>>3 % 10);
			return SYNC_ERROR;
		}

		edge_polarity = rxd & 1; // 0: positive edge, 1: negative edge
	}
	if (rxst_info->bit_tm + diff_time >= BIT_TIME) {
		rxst_info->onbit = 1;
		uint32_t addt = BIT_TIME - rxst_info->bit_tm;

		if (edge_polarity) {
			rxst_info->bit_sum += addt;
		}
		diff_time -= addt;

		int bit_level = rxst_info->bit_sum >= BIT_TIME2;

		rxst_info->bit_sum = 0;
		rxst_info->bit_tm = 0;

		int bit = !(bit_level ^ rxst_info->bit_level_prev); // decode NRZI
		rxst_info->bit_level_prev = bit_level;
		return bit;
	}
	rxst_info->onbit = 0;
	if (edge_polarity) rxst_info->bit_sum += diff_time;
    rxst_info->bit_tm += diff_time;

	return NO_SYNC;
}

/*
 * TNC receive task
 */
void rx_task(void *p)
{
    QueueHandle_t capqueue = *(QueueHandle_t *)p;

	code_info fx25_info;
	bitsync_info rxst_info;
	buffer_info buff_info;

 	init_bitsync_info(&rxst_info);
	init_buffer_info(&buff_info);

    for (;;) {

		//uint32_t t = rxd - rxd0;
		//int level = rxd & 1; // positive edge 0, negative edge 1
		//int bits = (t + BIT_TIME2) / BIT_TIME; // number of bit length between edges

		int bit = rx_bit_sync(capqueue, &rxst_info);
		if (bit == SYNC_ERROR) {
			init_code_info(&fx25_info);
			continue;
		}
		if (bit == NO_SYNC) {
			continue;
		}
		
		rx_bit_receive(bit, &buff_info, &fx25_info);
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
