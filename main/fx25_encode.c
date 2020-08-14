/*
 * send FX.25 / AX.25 packet to the air
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include "esp_log.h"

#include "rs.h"
//#include "afio.h"
#include "fx25tag.h"
#include "bit_stuffing.h"
#include "rmt.h"
#include "ax25.h"
#include "tx_cntl.h"

#define TAG "fx25_endcode"

/*
Tag_01 0xB74DB7DF8A532F3E RS(255, 239) 16-byte check value, 239 information bytes
Tag_02 0x26FF60A600CC8FDE RS(144,128) - shortened RS(255, 239), 128 info bytes
Tag_03 0xC7DC0508F3D9B09E RS(80,64) - shortened RS(255, 239), 64 info bytes
Tag_04 0x8F056EB4369660EE RS(48,32) - shortened RS(255, 239), 32 info bytes
Tag_05 0x6E260B1AC5835FAE RS(255, 223) 32-byte check value, 223 information bytes
Tag_06 0xFF94DC634F1CFF4E RS(160,128) - shortened RS(255, 223), 128 info bytes
Tag_07 0x1EB7B9CDBC09C00E RS(96,64) - shortened RS(255, 223), 64 info bytes
Tag_08 0xDBF869BD2DBB1776 RS(64,32) - shortened RS(255, 223), 32 info bytes
Tag_09 0x3ADB0C13DEAE2836 RS(255, 191) 64-byte check value, 191 information bytes
Tag_0A 0xAB69DB6A543188D6 RS(192, 128) - shortened RS(255, 191), 128 info bytes
Tag_0B 0x4A4ABEC4A724B796 RS(128, 64) - shortened RS(255, 191), 64 info bytes
 */

#define BUF_SIZE 1024
#define FX25_DATA_SIZE (4 + 8 + 255 + 1) // preamble + co_tag + RS_code + postamble
#define RS_CODE_SIZE 255
#define PARITY_SYMBOLS 16 // number of parity symbols, 16, 32, 64
#define TAG_SIZE 8

union CO_TAG {
    uint64_t val;
    uint8_t byte[sizeof(uint64_t)];
} tag = {
    .val = 0xB74DB7DF8A532F3E
};


typedef struct {
    uint8_t tags_byte[sizeof(uint64_t) * 2];
    int tags_byte_length;
    int block_number;
    int block_info_length;
    int block_code_length;
} encode_info;


#define AX25_FLAG 0x7e
#define FX25_FLAG 0x7e
#define FX25_PREAMBLE 4
#define FX25_POSTAMBLE 1

int choise_encode_info(encode_info *fx_code, int message_len, int parity_len)
{
    int i;
    int j;
    int k;

    for (i = CO_TAG_0B; i >= CO_TAG_01; --i) {
        if ((message_len <= tags[i].rs_info) && (parity_len == tags[i].rs_code - tags[i].rs_info)) {
            fx_code->block_number = 1;
            fx_code->block_code_length = tags[i].rs_code;
            fx_code->block_info_length = tags[i].rs_info;
            for (k = 0; k < TAG_SIZE; k++) {
                fx_code->tags_byte[k] = tags[i].byte[k];
            } 
            fx_code->tags_byte_length = TAG_SIZE;
            return 0;
        }
    }
    for (i = CO_TAG_0C; i <= CO_TAG_0F; i++) {
        if ((message_len <= tags[i].rs_info * tags[i].block_number) && (parity_len == tags[i].rs_code - tags[i].rs_info)) {
            fx_code->block_number = tags[i].block_number;
            fx_code->block_code_length = tags[i].rs_code;
            fx_code->block_info_length = tags[i].rs_info;
            for (k = 0; k < TAG_SIZE; k++) {
                fx_code->tags_byte[k] = tags[i].byte[k];
            } 
            fx_code->tags_byte_length = TAG_SIZE;
            return 0;
        }
    }
    for (i = CO_TAG_10; i <= CO_TAG_1F; i++) {
        if (message_len <= (RS_CODE_SIZE - parity_len) * tags[i].block_number) {
            fx_code->block_number = tags[i].block_number;

            for (j = CODE_TAG_01; j <= CODE_TAG_03; j++) {
                if (parity_len == code_tags[j].rs_code - code_tags[j].rs_info) {
                    fx_code->block_code_length = code_tags[j].rs_code;
                    fx_code->block_info_length = code_tags[j].rs_info;
                    for (k = 0; k < TAG_SIZE; k++) {
                        fx_code->tags_byte[k] = tags[i].byte[k];
                    }
                    for (k = 0; k < TAG_SIZE; k++) {
                        fx_code->tags_byte[k+TAG_SIZE] = code_tags[j].byte[k];
                    }
                    fx_code->tags_byte_length = TAG_SIZE*2;
                    return 0;
                }
            }
            ESP_LOGI(TAG, "fx25_encode(): unmached parity : len=%d, parity=%d", message_len, parity_len);
            return -1;
        }
    }
    ESP_LOGI(TAG, "fx25_encode(): packet too large: len=%d, parity=%d", message_len, parity_len);
    return -1;
}


int fx25_encode(uint8_t fx25_data[], int fx25_data_len, const uint8_t buf[], int info_len, int parity)
{
    int index;
    int i;
    uint8_t rs_buf[RS_CODE_SIZE];
    int offset;

    encode_info fx_info;

#if 0
    printf("fx25_encode(): buf[], info_len = %d\n", info_len);
    for (int i = 0; i < info_len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
#endif

    //  fx25tag_init();

    // find suitable encode info and TAG based on data length

    if (choise_encode_info(&fx_info, fx25_data_len, parity) < 0) {
        return -1;
    }


#if 0
    if (rs_init(RS_CODE_SIZE, RS_CODE_SIZE - parity) < 0) {
        fprintf(stderr, "rs_init error\n");
        return -1;
    }
#endif

    index = 0;
    //memset(fx25_data, AX25_FLAG, fx25_data_len);

#if 0
    printf("fx25_encode(): after memset(), buf[], info_len = %d\n", info_len);
    for (int i = 0; i < info_len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
#endif

    // preamble
    for (i = 0; i < FX25_PREAMBLE; i++) {
        fx25_data[index++] = FX25_FLAG;
    }

    // correlation tag
    for (i = 0; i < fx_info.tags_byte_length ; i++) {
        fx25_data[index++] = fx_info.tags_byte[i];
    }

    if (fx_info.block_number == 1) { // FX.25 specification

        // calculate RS parity
        offset = RS_CODE_SIZE - fx_info.block_code_length;
        bzero(rs_buf, offset);
        //bzero(rs_buf, RS_CODE_SIZE);
        //memset(&rs_buf[offset], AX25_FLAG, rs_info_byte);

        // copy data to RS work
        for (i = 0; i < info_len; i++) {
            rs_buf[offset + i] = buf[i];
        }

        // padding with AX25_FLAG
        memset(&rs_buf[offset + info_len], AX25_FLAG, fx_info.block_info_length - info_len);

        // generate RS parity
        if (rs_encode(rs_buf, RS_CODE_SIZE, RS_CODE_SIZE - parity) < 0) {
            fprintf(stderr, "rs_encode error\n");
            return -1;
        }

    #if 0
        printf("fx25_encode: rs_buf[], rs_code = %d, rs_info = %d, info_len = %d\n", rs_code_byte, rs_info_byte, info_len);
        for (i = 0; i < RS_CODE_SIZE; i++) {
            printf("%02x ", rs_buf[i]);
        }
        printf("\n");
    #endif

        // copy RS code
        for (i = 0; i < fx_info.block_code_length; i++) {
            fx25_data[index++] = rs_buf[offset + i];
        }

    } else {
        // JH1FBM extension
        int m = fx_info.block_number; // multiplier m = 2 ~
        for (int j = 0; j < m; j++) {

            // calculate RS parity
            //memset(&rs_buf[0], AX25_FLAG, RS_CODE_SIZE);

            // copy data to RS work
            for (i = 0; i < fx_info.block_info_length; i++) {
                rs_buf[i] = (i * m + j < info_len) ? buf[i * m + j] : AX25_FLAG;
            }

            // generate RS parity
            if (rs_encode(rs_buf, RS_CODE_SIZE, RS_CODE_SIZE - parity) < 0) {
                fprintf(stderr, "rs_encode error\n");
                return -1;
            }

            // copy RS code
            for (i = 0; i < RS_CODE_SIZE; i++) {
                fx25_data[i * m + j + index] = rs_buf[i];
            }
        }
    }

    // postamble
    for (i = 0; i < FX25_POSTAMBLE; i++) {
        fx25_data[index++] = FX25_FLAG;
    }

    return index;
}

#define MAX_PKT_SIZE 2048
#define MIN_PKT_SIZE (7 + 7 + 1 + 1)
#define AX25_BUF_SIZE (MAX_PKT_SIZE + 2)
#define BITS_BUF_SIZE 2456
#define FX25_BUF_SIZE (4 + 8*2 + 256*15 + 2)

#ifdef CONFIG_TNC_STATISTICS
uint8_t source_callsign[7] = { 'N' << 1, 'O' << 1, 'C' << 1, 'A' << 1, 'L' << 1, 'L' << 1, 0 };
#endif

void fx25_send_packet(uint8_t buf[], int size, int wait, int tnc_mode)
{
    static uint8_t ax25_buf[AX25_BUF_SIZE];
    static uint8_t bits_buf[BITS_BUF_SIZE];
    static uint8_t fx25_buf[FX25_BUF_SIZE];
    int fcs;
    int len;
    int bits_len;
    int fx25_len;
    int ax25 = tnc_mode == AX25_MODE;
    int parity = tnc_mode;

    ESP_LOGI(TAG, "fx25_send_packet(): tnc_mode=%d, ax25=%d, parity=%d", tnc_mode, ax25, parity);

    if (buf == NULL) return;;
    if (size < MIN_PKT_SIZE) return;
    if (size > MAX_PKT_SIZE) return;

#ifdef CONFIG_TNC_STATISTICS
    // save source callsgin
    memcpy(source_callsign, &buf[7], 7);
#endif

    memcpy(ax25_buf, buf, size);
    len = size;

    fcs = ax25_fcs(ax25_buf, len);
    ax25_buf[len++] = fcs & 0xff;
    ax25_buf[len++] = fcs >> 8;
    
    bits_len = bitstuff_encode(bits_buf, BITS_BUF_SIZE, ax25_buf, len);

    if (ax25) {
	      send_packet(bits_buf, bits_len, wait); // send AX.25 packet
    } else {
      	// send FX.25 packet
        fx25_len = fx25_encode(fx25_buf, FX25_BUF_SIZE, bits_buf, bits_len, parity);
        send_packet(fx25_buf, fx25_len, wait);
    }
}

#ifdef notdef
int main(int argc, char *argv[])
{
    FILE *ifp = stdin;
    FILE *ofp = stdout;
    uint8_t buf[BUF_SIZE];
    uint8_t fx25_data[FX25_DATA_SIZE];
    int bit_length;
    int byte_size;
    int fx25_len;

    while ((bit_length = afio_read(ifp, buf, BUF_SIZE)) >= 0) {
        byte_size = (bit_length + 7) / 8;
        fx25_len = fx25_encode(fx25_data, buf, byte_size, PARITY_SYMBOLS);
        if (fx25_len > 0) afio_write(ofp, fx25_data, fx25_len, 0);
    }

    return 0;
}
#endif
