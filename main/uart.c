/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "rmt.h"
#include "fx25_encode.h"
#include "tx_cntl.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define RX_BUF_SIZE (1024*32)
#define TX_BUF_SIZE (1024*1)

#define UART_QUEUE_SIZE 32
#ifdef CONFIG_KISS_BAUD_RATE
#define UART_BAUD_RATE CONFIG_KISS_BAUD_RATE
#else
#define UART_BAUD_RATE 115200
#endif
//#define UART_BAUD_RATE 1200
#define UART_RINGBUF_SIZE (1024*32)

#define AF_BUF_SIZE (1024+1)	// +1 is for "type"
#define UART_BUF_SIZE 64
#define FX25_BUF_SIZE (4 + 8 + 255*5 + 1)	// FX.25 buffer size
#define AX25_BUF_SIZE (1024 + 2)		// data + FCS
#define MAX_PKT_SIZE 1024

static QueueHandle_t uart0_queue;
static RingbufHandle_t uart0_ringbuf;

enum COMMAND {
    CMD_DATA = 0,
    CMD_TXDELAY,
    CMD_P,
    CMD_SLOTTIME,
    CMD_TXTAIL,
    CMD_FULLDUPLEX,
    CMD_SETHARDWARE,
};

// stub
//void set_txdelay(int ch, int param) {}
//void set_persistence(int ch, int param) {}
//void set_slottime(int ch, int param) {}
//void set_fullduplex(int ch, int param) {}
//void set_sethardware(int ch, int param) {}

// process input packet
void input_packet(uint8_t buf[], int size)
{
    uint8_t type;
    int cmd;
    int ch;
    int param;

    if ((buf == NULL) || (size <= 0)) return;

    // first byte is type indicator
    type = buf[0];
    cmd = type & 0x0f;
    ch = type >> 4;
    param = buf[1];
    int tnc_mode;
    
    switch (cmd) {
	case CMD_DATA:
	    // delete first byte, the byte is KISS port number
	    // param: buf, size, wait for Queuing, ax25 flag, FX25 parity
	    if (size > 1) {
    		tnc_mode = get_tnc_mode();
		switch (ch) {
		    case 0: // AX.25 or FX.25 depend on the setting
	        	fx25_send_packet(&buf[1], size - 1, 0, tnc_mode);
			break;
		    case 1: // AX.25
	        	fx25_send_packet(&buf[1], size - 1, 0, AX25_MODE);
			break;
		    case 2: // FX.25 parity 16 symbols
	        	fx25_send_packet(&buf[1], size - 1, 0, FX25_PARITY_16);
			break;
		    case 3: // FX.25 parity 32 symbols
	        	fx25_send_packet(&buf[1], size - 1, 0, FX25_PARITY_32);
			break;
		    case 4: // FX.25 parity 64 symbols
	        	fx25_send_packet(&buf[1], size - 1, 0, FX25_PARITY_64);
			break;
		}
	    }
	    break;

	case CMD_TXDELAY:
	    set_txdelay(ch, param);
	    break;

	case CMD_P:
	    set_parameter_p(ch, param);
	    break;

	case CMD_SLOTTIME:
	    set_slottime(ch, param);
	    break;

	case CMD_FULLDUPLEX:
	    set_fullduplex(ch, param);
	    break;

	case CMD_SETHARDWARE:
	    set_sethardware(ch, &buf[1], size - 1);
    }
}

enum AF_STATE {
    AF_IDLE = 0,
    AF_FEND,
    AF_FESC,
};

// KISS mode asynchronouse frame format
#define FEND 0xc0	// frame end
#define FESC 0xdb	// frame escape
#define TFEND 0xdc	// transposed frame end
#define TFESC 0xdd	// transposed trame escape

void uart_af_read(int num, int size)
{
    static uint8_t af_buf[AF_BUF_SIZE];
    static uint8_t uart_buf[UART_BUF_SIZE];
    int len = size;
    int i;
    static int ai = 0;
    uint8_t c;
    int d;
    static enum AF_STATE state = AF_IDLE;

    ESP_LOGI(TAG, "uart_af_read: state = %d", state);

    if (size <= 0) {	// reset internal state
	state = AF_IDLE;
	ai = 0;

	return;
    }

    while (size > 0) {
	if (len > UART_BUF_SIZE) len = UART_BUF_SIZE;

        len = uart_read_bytes(num, uart_buf, len, 0);
	ESP_LOGI(TAG, "uart_read_bytes: len = %d", len);

	if (len <= 0) {		// some error occured
	    uart_flush_input(num);
	    return;
	}

        for (i = 0; i < len; i++) {
	    c = uart_buf[i];
	    d = -1;
	    //ESP_LOGI(TAG, "uart_buf[%d] = %02x", i, c);

	    switch (state) {
		case AF_IDLE:	// between async frame
		    if (c == FEND) { // start of async frame
		        state = AF_FEND;
			ai = 0;
			ESP_LOGI(TAG, "FEND");
		    }
		    break;

	        case AF_FEND:	// process async frame data
		    switch (c) {
		        case FEND:	// end of async frame
			    ESP_LOGI(TAG, "FEND: ai = %d", ai);
		            if (ai > 0) {
				input_packet(af_buf, ai); // process input packet
				ESP_LOGI(TAG, "input_packet: ai = %d", ai);
				state = AF_IDLE;
			    }
			    break;
			case FESC:	// enter escaped mode
			    state = AF_FESC;
			    break;
			default:	// normal data
			    d = c;
		    }
		    break;

	        case AF_FESC:	// escaped mode
		    switch (c) {
		        case TFEND:
			    d = FEND;	// escaped data
			    break;
			case TFESC:
			    d = FESC;	// escaped data
		    }
		    state = AF_FEND; // exit escaped mode
	    }

	    if (d >= 0) { // add data to buffer
		if (ai < AF_BUF_SIZE) {
		    af_buf[ai++] = d;
		} else {
		    state = AF_IDLE; // async frame too large, discard packet
		}
	    }

	}
	size -= len;
    }

    ESP_LOGI(TAG, "uart_af_read: return, state = %d, ai = %d", state, ai);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    //size_t buffered_size;
    //uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            //bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
		    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_af_read(EX_UART_NUM, event.size);
		    ESP_LOGI(TAG, "[UART EVT]:");
                    break;

                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
		    uart_af_read(EX_UART_NUM, -1); // reset
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
		    uart_af_read(EX_UART_NUM, -1); // reset
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;

                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;

                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;

#if 0
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
#endif
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    //free(dtmp);
    //dtmp = NULL;
    vTaskDelete(NULL);
}

#define UART_NUM UART_NUM_0
//#define PRINT_ESC 1	// escape non-printable character for debug

static int timer(int stop)
{
    static struct timeval tv0, tv1;

    if (!stop) {
	gettimeofday(&tv0, NULL);
	return 0;
    }
    
    gettimeofday(&tv1, NULL);

    int sec = tv1.tv_sec - tv0.tv_sec;
    int usec = tv1.tv_usec - tv0.tv_usec;

    return sec * 1000000 + usec;
}

#define AF_TX_BUF_SIZE 256
#define UART_TAG "uart"

static void uart_send_split(uint8_t *item[], size_t size[])
{
    timer(0);

    static char af_buf[AF_TX_BUF_SIZE];
    int ai = 0;
    int bi = 0;
    uint8_t c;
    uint8_t next_byte = 0;
    int usec;
    uint8_t *buf;
    int i;
    int len;
    int total = 0;

#ifndef CONFIG_TNC_DEMO_MODE
#ifdef PRINT_ESC
    af_buf[ai++] = 'F';
    af_buf[ai++] = '0';
#else
    af_buf[ai++] = FEND;
    af_buf[ai++] = 0x00; // channel 0, data frame
#endif
#endif

    for (i = 0; i < 2; i++) {

	buf = item[i];
	if (buf == NULL) break;
	len = size[i];
	total += len;
	bi = 0;

        while (bi < len) {
       
	    if (next_byte) {
	        c = next_byte;
	        next_byte = 0;
	    } else {
	        c = buf[bi];
	    }

#ifdef CONFIG_TNC_DEMO_MODE

#define LF '\n'
#define CR '\r'

	    if (c == LF) {
		c = LF;
		next_byte = CR;
	    } else {
		bi++;
	    }
#else
	    if (c == FEND) {
	        c = FESC;
	        next_byte = TFEND;
	    } else if (c == FESC) {
	        c = FESC;
		next_byte = TFESC;
	    } else {
	        bi++;
	    }
#endif

#ifdef PRINT_ESC
	    af_buf[ai++] = isprint(c) ? c : '.';
#else
	    af_buf[ai++] = c;
#endif
	    if (ai >= AF_TX_BUF_SIZE) {
	        uart_write_bytes(UART_NUM, af_buf, ai);
	        ai = 0;
	    }
	}
    }

#ifndef CONFIG_TNC_DEMO_MODE
#ifdef PRINT_ESC
    af_buf[ai++] = 'F';
#else
    af_buf[ai++] = FEND;
#endif
#endif

    uart_write_bytes(UART_NUM, af_buf, ai);

#ifdef PRINT_ESC
    uart_write_bytes(UART_NUM, "\r\n", 2);
#endif

    usec = timer(1);
    if (usec > 10000) ESP_LOGD(UART_TAG, "uart_send(bytes = %d): %d us", total, usec);

    return;
}

static void uart_send_task(void *p)
{
    RingbufHandle_t rb = (RingbufHandle_t)p;
    uint8_t *item[2];
    size_t size[2];

    while (1) {
	if (xRingbufferReceiveSplit(rb, (void **)&item[0], (void **)&item[1], &size[0], &size[1], portMAX_DELAY) == pdTRUE) {
	    
	    if (item[0]) {
		uart_send_split(item, size);
		vRingbufferReturnItem(rb, item[0]);
		if (item[1]) vRingbufferReturnItem(rb, item[1]);
	    }
	}
    }
}

#define RINGBUF_N 4

static RingbufHandle_t tcp_ringbuf[RINGBUF_N];

RingbufHandle_t *uart_add_ringbuf(RingbufHandle_t ringbuf)
{
    for (int i = 0; i < RINGBUF_N; i++) {
	if (tcp_ringbuf[i] == NULL) {
	    tcp_ringbuf[i] = ringbuf;
	    return &tcp_ringbuf[i];
	}
    }

    return NULL;
}

void uart_delete_ringbuf(RingbufHandle_t ringbuf)
{
    for (int i = 0; i < RINGBUF_N; i++) {
	if (tcp_ringbuf[i] == ringbuf) {
	    tcp_ringbuf[i] = NULL;
	}
    }
}

void packet_output(uint8_t buf[], int size)
{
    if ((buf == NULL) || (size <= 0)) return;

    if (xRingbufferSend(uart0_ringbuf, buf, size, 0) != pdTRUE) {
	ESP_LOGD(UART_TAG, "uart ring buffer full: %d bytes", size);
    }
    for (int i = 0; i < RINGBUF_N; i++) {
	if (tcp_ringbuf[i] && xRingbufferSend(tcp_ringbuf[i], buf, size, 0) != pdTRUE) { // 0: no wait
	    ESP_LOGD(UART_TAG, "tcp ring buffer[%d] full: %d bytes", i, size);
	}
    }
}

void uart_init(void)
{
    //esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));

    //Set UART log level
    //esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, RX_BUF_SIZE, TX_BUF_SIZE, UART_QUEUE_SIZE, &uart0_queue, 0));

    //Set uart pattern detect function.
    //uart_enable_pattern_det_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 10000, 10, 10);
    //Reset the pattern queue length to record at most 20 pattern positions.
    //uart_pattern_queue_reset(EX_UART_NUM, 20);

    //Create a task to handler UART event from ISR
    if (xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL) != pdPASS) {
	ESP_LOGE(TAG, "uart_event_task creation fail");
	abort();
    }

    uart0_ringbuf = xRingbufferCreate(UART_RINGBUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
    if (uart0_ringbuf == NULL) {
	ESP_LOGD(TAG, "can not allocate ring buffer");
	abort();
    }
    if (xTaskCreate(uart_send_task, "uart_send_task", 2048, uart0_ringbuf, 12, NULL) != pdPASS) {
	ESP_LOGE(TAG, "uart_send_task creation fail");
	abort();
    }
}
