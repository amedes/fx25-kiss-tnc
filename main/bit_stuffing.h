/*
 * bit_stuffing.h
 *
 * add and delete bit stuffing bit
 */

int bitstuff_encode(uint8_t bits_buf[], int bits_len, uint8_t buf[], int len);
int bitstuff_decode(uint8_t byte_buf[], int buf_len, uint8_t bits_buf[], int bits_len);
