/*
 * ax25_decode.h
 */
int ax25_decode(int bits, uint8_t buf[], int len);
int ax25_decode_bit(int bit, uint8_t buf[], int len);
int ax25_nrzi(int bits, uint8_t buf[], int len);
int ax25_nrzi_bit(int level, uint8_t buf[], int len);
int nrzi_decode(uint8_t buf[], int len, uint8_t nrzi[], int nrzi_bitlen);
int ax25_bec(uint8_t ax25_buf[], int ax25_len, uint8_t nrzi_buf[], int nrzi_len);
