#ifndef FX25_CODE_INFO_H

#include <stdint.h>

typedef struct {
    uint8_t tags_byte[sizeof(uint64_t) * 2];
    int tags_byte_length;
    int block_number;
    int block_info_length;
    int block_code_length;
} code_info;

int choise_decode_info(code_info *fx_code, int bit);

#define FX25_CODE_INFO_H
#endif
