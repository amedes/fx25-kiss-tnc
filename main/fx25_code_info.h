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
    int buff_size;
    int data_size;
    int bit_pos;
} buffer_info;

typedef struct {
    uint32_t rxd0;
	uint32_t rxd;
    int bit_tm;
    int bit_sum;
    int edge_level_prev;
    int bit_level_prev;
} bitsync_info;

#define FRAME_CONTINUE 0
#define PN_SYNC_DONE 1
#define FRAME_ENDED 2
#define FRAME_ERROR -1
#define UNDEFINED_PN -2
#define BUFF_NOT_ENOUGH -3

void clear_code_info(code_info *fx_code);
int choise_decode_info(code_info *fx_code, int bit);

#define FX25_CODE_INFO_H
#endif
