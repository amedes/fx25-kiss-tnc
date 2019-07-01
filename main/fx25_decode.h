/*
 * fx25_decode()
 * decode FX.25 packet.
 * bits is length of NRZI mark or space.
 */
int fx25_decode(int bits, uint8_t ax25_buf[], int buf_len, int *error_correction);
int fx25_decode_bit(int bit, uint8_t ax25_buf[], int buf_len);
int fx25_rsdecode(uint8_t fx25_buf[], int tag_no);
