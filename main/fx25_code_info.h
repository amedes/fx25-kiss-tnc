#ifndef FX25_CODE_INFO_H

#include <stdint.h>

#define MAX_SYNC_TAG 2

typedef struct {
    union {
        uint8_t tags_byte[sizeof(uint64_t) * MAX_SYNC_TAG];
        uint64_t tagss[MAX_SYNC_TAG];
    };
    int tags_byte_length;
    int block_number;
    int block_info_length;
    int block_code_length;
    int sync_state;
} code_info;

typedef struct {
    uint8_t *buff;
    int size;
    int bit_pos;
} buffer_info;

void clear_code_info(code_info *fx_code);
int choise_decode_info(code_info *fx_code, int bit);

#define FX25_CODE_INFO_H
#endif
