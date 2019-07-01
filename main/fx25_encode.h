int fx25_encode(uint8_t fx25_data[], int fx25_data_len, const uint8_t buf[], int info_len, int parity);
void fx25_send_packet(uint8_t buf[], int size, int wait, int tnc_mode);
