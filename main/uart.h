/*
 * uart.h
 */
void uart_init(void);
void packet_output(uint8_t buf[], int size);
void input_packet(uint8_t buf[], int size);
RingbufHandle_t *uart_add_ringbuf(RingbufHandle_t ringbuf);
void uart_delete_ringbuf(RingbufHandle_t ringbuf);
