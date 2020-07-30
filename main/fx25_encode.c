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

union CO_TAG {
  uint64_t val;
  uint8_t byte[sizeof(uint64_t)];
} tag = {
  .val = 0xB74DB7DF8A532F3E
};

#define AX25_FLAG 0x7e
#define FX25_FLAG 0x7e
#define FX25_PREAMBLE 4
#define FX25_POSTAMBLE 1

int fx25_encode(uint8_t fx25_data[], int fx25_data_len, const uint8_t buf[], int info_len, int parity)
{
  int rs_code_byte;
  int rs_info_byte;
  int index;
  int tag_no = 0;
  int code_tag_no = 0;
  int i;
  uint8_t rs_buf[RS_CODE_SIZE];
  int offset;
  int need_flame = 1;

#if 0
  printf("fx25_encode(): buf[], info_len = %d\n", info_len);
  for (int i = 0; i < info_len; i++) {
	  printf("%02x ", buf[i]);
  }
  printf("\n");
#endif

  //  fx25tag_init();

  // find suitable CO TAG based on data length
  for (i = CO_TAG_0B; i >= CO_TAG_01; --i) {
    if ((info_len <= tags[i].rs_info) && (parity == tags[i].rs_code - tags[i].rs_info)) {
      tag_no = i;
      need_flame = 1;
      rs_code_byte = tags[tag_no].rs_code;
      rs_info_byte = tags[tag_no].rs_info;
      break;
    }
  }

// #define RS_INFO_SIZE 239
// parity = FX25_PARITY_16;

  // JH1FBM extension type 1
  if (tag_no == 0) {
    for (i = CO_TAG_0C; i <= CO_TAG_0F; i++) {
      if ((info_len <= tags[i].rs_info * tags[i].flame_number) 
          && (parity == tags[i].rs_code - tags[i].rs_info)) {
        tag_no = i;
        need_flame = tags[i].flame_number;
        rs_code_byte = tags[tag_no].rs_code;
        rs_info_byte = tags[tag_no].rs_info;
        break;
      }
    }
  }

  // JH1FBM extension type 2
  if (tag_no == 0) {
    need_flame = info_len / (RS_CODE_SIZE - parity);
    if (need_flame > 1 && need_flame < 16) {
      tag_no = need_flame + CO_TAG_10;
      for (i = CODE_TAG_01; i <= CODE_TAG_03; i++) {
        if (parity == code_tags[i].rs_code - code_tags[i].rs_info) {
          code_tag_no = i;
          rs_code_byte = code_tags[code_tag_no].rs_code;
          rs_info_byte = code_tags[code_tag_no].rs_info;
        }
      }
      if (code_tag_no == 0) { // suitable CODE TAG not found
        ESP_LOGI(TAG, "fx25_encode(): unmached parity : len=%d, parity=%d", info_len, parity);
        return -1;
      }
    }
  }
  //printf("fx25_encode(): tag_no = %02x\n", tag_no);

  if (tag_no == 0) { // suitable TAG not found

    ESP_LOGI(TAG, "fx25_encode(): packet too large: len=%d, parity=%d", info_len, parity);
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
  for (i = 0;i < sizeof(tag); i++) {
    fx25_data[index++] = tags[tag_no].byte[i];
  }

  if (tag_no <= CO_TAG_0B) { // FX.25 specification

    // calculate RS parity
    offset = RS_CODE_SIZE - rs_code_byte;
    bzero(rs_buf, offset);
    //bzero(rs_buf, RS_CODE_SIZE);
    //memset(&rs_buf[offset], AX25_FLAG, rs_info_byte);

    // copy data to RS work
    for (i = 0; i < info_len; i++) {
      rs_buf[offset + i] = buf[i];
    }

    // padding with AX25_FLAG
    memset(&rs_buf[offset + info_len], AX25_FLAG, rs_info_byte - info_len);

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
    for (i = 0; i < rs_code_byte; i++) {
      fx25_data[index++] = rs_buf[offset + i];
    }

  } else {
    // JH1FBM extension
    if (tag_no > CO_TAG_0F) {
        // JH1FBM extension 2
      for (i = 0;i < sizeof(tag); i++) {
        fx25_data[index++] = code_tags[code_tag_no].byte[i];
      }
    }

    int m = need_flame; // multiplier m = 2 ~
    for (int j = 0; j < m; j++) {

      // calculate RS parity
      //memset(&rs_buf[0], AX25_FLAG, RS_CODE_SIZE);

      // copy data to RS work
      for (i = 0; i < rs_info_byte; i++) {
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
