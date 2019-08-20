#include <stdio.h>
#include <stdint.h>
//#include <strings.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>

#include "esp_log.h"

//#include <soc/rtc_cntl_reg.h>

#include "modem.h"
#include "fx25tag.h"
#include "bit_stuffing.h"
#include "ax25.h"
#include "rs.h"
#include "config.h"

#define STATE_SEARCH_TAG 1
#define STATE_DATA 2

#define BIT_TIME (80*1000*1000 / BAUD_RATE)
#define BIT_LEN_MAX 32

#define BUF_SIZE (255*5) // FX.25 decode buffer
#define TAG_BYTE_LEN 8
#define FX25_FLAG 0x7e
#define AX25_FLAG 0x7e

#define FX25_CORRELATION_CNT 8

#define RS_CODE_SIZE 255

#define TAG "fx25_decode"

#ifdef CONFIG_TNC_DEMO_MODE
int tag_error_pkts = 0;
#endif

#if 0
static const int BITS_COUNT_TABLE[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};

static inline int bit_count(uint64_t bits)
{
    int n = 0;
    uint8_t *p = (uint8_t *)&bits;

    n += BITS_COUNT_TABLE[*p++];
    n += BITS_COUNT_TABLE[*p++];
    n += BITS_COUNT_TABLE[*p++];
    n += BITS_COUNT_TABLE[*p++];
    n += BITS_COUNT_TABLE[*p++];
    n += BITS_COUNT_TABLE[*p++];
    n += BITS_COUNT_TABLE[*p++];
    n += BITS_COUNT_TABLE[*p];

    return n;
}

#else

static inline int bit_count(uint64_t bits)
{
    uint64_t b64_0 = bits, b64_1;
    uint32_t b32_0, b32_1;
    uint16_t b16_0, b16_1;
    uint8_t b8_0, b8_1;

    b64_1 = (b64_0 >> 1) & 0x5555555555555555LLU;
    b64_0 &=               0x5555555555555555LLU;

    b64_0 += b64_1; /* 01 + 01 = 10 */

    b64_1 = (b64_0 >> 2) & 0x3333333333333333LLU;
    b64_0 &=               0x3333333333333333LLU;

    b64_0 += b64_1; /* 10 + 10 = 0100 */

    b32_1 = (b64_0 >> 32); /* higher 32bit */
    b32_0 = b64_0;         /* lower 32bit */

    b32_0 += b32_1; /* 0100 + 0100 = 1000 */

    b32_1 = (b32_0 >> 4) & 0x0f0f0f0f; /* separate each 4bit */
    b32_0 &=               0x0f0f0f0f;

    b32_0 += b32_1; /* 0x8 + 0x8 = 0x10 */

    b16_1 = (b32_0 >> 16); /* higher 16bit */
    b16_0 = b32_0; 	   /* lower 16bit */

    b16_0 += b16_1; /* 0x10 + 0x10 = 0x20 */

    b8_1 = (b16_0 >> 8); /* higher 8bit */
    b8_0 = b16_0;	 /* lower 8bit */

    b8_0 += b8_1; /* 0x20 + 0x20 = 0x40 */

    return b8_0;
}
#endif

int fx25_search_tag(uint64_t *correlation_tag, int data_bit)
{
  int count;
  int i;
  uint64_t bits;

  /* add 1 bit to tag data */
  *correlation_tag >>= 1;
  *correlation_tag |= ((uint64_t)data_bit << 63);

  /* compare tag with TAG_01 to TAG_0F */
  for (i = CO_TAG_01; i <= CO_TAG_0F; i++) {

    bits = *correlation_tag ^ tags[i].tag;
    count = bits ? bit_count(bits) : 0; // set bits if different 

    if (count <= FX25_CORRELATION_CNT) {

      if (count != 0) {
        ESP_LOGI(TAG, "bit_count(%016llx) = %d", bits, count);
        ESP_LOGI(TAG, "%016llx\n%016llx\n", *correlation_tag, tags[i].tag);
      }

      return i; // find i-th TAG
    }
#ifdef CONFIG_TNC_DEMO_MODE 
    else if (count <= 12) {
      printf("\tFX25 info: Tag error %d bits, bit pattern: %016llx\n", count, bits);
      tag_error_pkts++;
#if 0
      fprintf(stderr, "fx25_decode: correlation tag match %d bits\n", 64 - count);
      fprintf(stderr, "bit_count(%016llx) = %d\n", bits, count);
      fprintf(stderr, "%016llx\n%016llx\n", *correlation_tag, tags[i].tag);
#endif
    }
#endif

  }

  return -1; // not found tag
}

/*
 * fx25_decode()
 * bits: length of NRZIed mark or spece time
 */
int fx25_decode(int bits, uint8_t ax25_buf[], int ax25_buf_size, int *rs_status)
{
  static int state = STATE_SEARCH_TAG;
  static uint8_t buf[BUF_SIZE];
  static uint64_t fx25tag = 0;
  static int tag_no;
  static int codeblock_bits;
  int level;
  static int bit_offset;
  static int initialized = 0;
  int i;
  int ax25_len;
  int fcs_ok = 0;
  int get_packet = 0;
  //static uint8_t ax25_buf[BUF_SIZE];

  // initialize tag value
  if (!initialized) {
    fx25tag_init();
    initialized = 1;
  }

  if ((bits <= 0) || (bits >= BIT_LEN_MAX)) {
    state = STATE_SEARCH_TAG;
    fx25tag = ~0LLU; // set all 1
    ESP_LOGI(TAG, "FX25 decode: wrong bits: %d, bit_offset = %d\n", bits, bit_offset);

    return 0;
  }

  if (rs_status) *rs_status = 0; // clear rs_decode

  level = 0;
  for (i = 0; i < bits; i++) { // loop bits times, bit pattern is "0111..."

      switch (state) {
      case STATE_SEARCH_TAG:

	if ((tag_no = fx25_search_tag(&fx25tag, level)) > 0) { // correlation tag found
	  state = STATE_DATA;
	  codeblock_bits = tags[tag_no].rs_code * 8; // RS CODEBLOCK bit length
	  bzero(buf, BUF_SIZE);
	  ESP_LOGI(TAG, "fx25 tag: bit_offset = %d", bit_offset);
	  bit_offset = 0;
	  tag_t *tp = &tags[tag_no];
	  ESP_LOGI(TAG, "found fx25 tag: %02x, (%d, %d), %d", tag_no, tp->rs_code, tp->rs_info, codeblock_bits);
	}

	break;

      case STATE_DATA:
	if (bit_offset < BUF_SIZE * 8) {
	  if (level)
		buf[bit_offset / 8] |=   1 << (bit_offset % 8);
	  else
		buf[bit_offset / 8] &= ~(1 << (bit_offset % 8));
	}
	bit_offset++;

	if (bit_offset >= codeblock_bits) {
	  get_packet = 1;

	  // send fx.25 packet to next process
	  //afio_write(ofp, buf, bit_offset / 8, 0);
	  ESP_LOGI(TAG, "get fx25 packet: bit length: %d", bit_offset);
#if 0
	  for (int j = 0; j < bit_offset/8; j++) {
	    printf("%02x ", buf[j]);
	  }
	  printf("\n");
	  int j;
	  for (j = 0; j < bit_offset/8; j++) {
	    int c = buf[j];
	    printf("%c", (c >= ' ' && c <= '~') ? c : '.');
	  }
	  printf("\n");
#endif

	  int rs_decode_done = 0;
	  static int rs_success = 0;
	  static int rs_decode_cnt = 0;

	  do {

	    ax25_len = bitstuff_decode(ax25_buf, ax25_buf_size, &buf[1], bit_offset/8 - 1); // skip AX25 flag
	    ESP_LOGI(TAG, "ax25_len = %d", ax25_len);
	    if (ax25_len > 2) {
	      int fcs = ax25_fcs(ax25_buf, ax25_len - 2);
	      int fcs2 = ax25_buf[ax25_len - 2] | (ax25_buf[ax25_len - 1] << 8);

	      ESP_LOGI(TAG, "FCS: %04x", fcs);
#if 0
	      for (int j = 0; j < ax25_len; j++) {
	        printf("%02x ", ax25_buf[j]);
	      }
	      printf("\n");
#endif
	      if (fcs == fcs2) {
	        ESP_LOGI(TAG, "FCS is correct");
		fcs_ok = 1;
		if (rs_decode_done) {
		  rs_success++;
		  ESP_LOGI(TAG, "rs_decode success: %d / %d", rs_success, rs_decode_cnt);
		}

		ESP_LOGI(TAG, "FX25 correct data: rs_decode = %d", rs_decode_cnt);
#if 0
		for (int j = 0; j < ax25_len - 2; j++) {
		  int c = ax25_buf[j];

		  if (j < 14) c >>= 1;
		  printf("%c", isprint(c) ? c : '.');
		}
		printf("\n");
#endif
		break;

	      } else {
	        ESP_LOGI(TAG, "FCS error");
	      }

	    } else {
	      ESP_LOGI(TAG, "un-bit stuffing error");
	    }

	    if (!fcs_ok) {
	      uint8_t rs_buf[RS_CODE_SIZE];
	      int rs_code_size = tags[tag_no].rs_code;
	      int rs_info_size = tags[tag_no].rs_info;
	      int offset = RS_CODE_SIZE - rs_code_size;
	      int parity = rs_code_size - rs_info_size;
	      int rs_err = 0;

	      // RS decode
	      if (rs_decode_done) break;

	      static uint8_t pre_rs_decode[BUF_SIZE];

	      buf[0] = AX25_FLAG; // fixed value, avoid error correction
	      memcpy(pre_rs_decode, buf, rs_code_size);

	      if (tag_no <= CO_TAG_0B) { // FX.25 defined TAG

	      // buffer clear
	      bzero(rs_buf, RS_CODE_SIZE);

	      // copy RS code block
	      memcpy(rs_buf + offset, buf, rs_code_size);

	      
	      rs_err = rs_decode(rs_buf, RS_CODE_SIZE, RS_CODE_SIZE - parity);


	      // copy rs decode code
	      memcpy(buf, rs_buf + offset, rs_info_size);

	      } else if ((tag_no >= CO_TAG_0C) && (tag_no <= CO_TAG_0F)) {
		// Nemoto extension

	        // copy RS code block
	        buf[0] = AX25_FLAG; // fixed value, avoid error correction

		int m =	rs_code_size / RS_CODE_SIZE;
		int rs_sum = 0;
		for (int j = 0; j < m; j++) {
		  for (int i = 0; i < RS_CODE_SIZE; i++) {
		     rs_buf[i] = buf[i * m + j];
		  }

#define RS_INFO_SIZE 239

	          rs_err = rs_decode(rs_buf, RS_CODE_SIZE, RS_INFO_SIZE);

		  if (rs_err > 0) { // decode success
		     rs_sum += rs_err;

		     // copy RS decode data
		     for (int i = 0; i < RS_INFO_SIZE; i++) {
		       buf[i * m + j] = rs_buf[i];
		     }

		  } else if (rs_err < 0) { // decode error
		    rs_sum = rs_err;
		    break;
		  }
		}

		rs_err = rs_sum;
		offset = 0;
	      }

	      if (rs_status) *rs_status = rs_err;
	      rs_decode_done = 1;
	      rs_decode_cnt++;
	      ESP_LOGI(TAG, "rs_decode: %d", rs_err);

	      if (rs_err < 0) {
	        ESP_LOGI(TAG, "RS decode error: %d", rs_err);
	        break;
	      }

	      ESP_LOGI(TAG, "RS decode result: %d", rs_err);
#if 0
	      for (int i = 0; i < rs_info_size; i++) {
	        int c = pre_rs_decode[i];
	        int d = buf[i];

	        if (c == d) printf("%02x ", c);
	        else printf("(%02x->%02x) ", c, d);
	      }
	      printf("\n");
#endif
	    }
	  	
	  } while (!fcs_ok);

	  state = STATE_SEARCH_TAG;
	  fx25tag = 0;
	}
      }

    level = 1;
  }

  if (fcs_ok) return ax25_len;
  if (get_packet) return -1; // discard data due to error

  return 0;
}

/*
 * fx25_decode_bit: input "bit" means one bit (0 or 1) of data
 *
 * return: tag_no of FX.25 packet, if read all packet data
 */
int fx25_decode_bit(int level, uint8_t fx25_buf[], int fx25_buf_size)
{
  static int level_prev = 1;
  static int state = STATE_SEARCH_TAG;
  static uint64_t fx25tag = 0;
  static int tag_no;
  static int codeblock_bits;
  static int bit_offset;
  static uint16_t fx25flag = 0;
  int rs_code_size;
  int bit;
  int flag_found = 0;

#if 0
  // initialize tag value
  if (!initialized) {
    fx25tag_init();
    initialized = 1;
  }
#endif

  if (level < 0) { // reset variables
    state = STATE_SEARCH_TAG;
    fx25tag = 0;
    //printf("fx25_decode_bit(): level_prev = %d, bit_offset = %d\n", level_prev, bit_offset);
    level_prev = 1;
    fx25flag = 0;

    return 0;
  }

  bit = !(level ^ level_prev); // decode NRZI
  level_prev = level;

      switch (state) {
      case STATE_SEARCH_TAG:

	//printf("fx25_decode_bit(): fx25tag = %016llx, bit = %d\n", fx25tag, bit);

//#define FX25_FLAGS 0x7e7e
#define FX25_FLAGS 0x7e
	
	// flag detection for bit sync
	fx25flag >>= 1;
	//if (bit) fx25flag |= 0x8000;
	if (bit) fx25flag |= 0x80;
	if (fx25flag == FX25_FLAGS) {
	    flag_found = 1;
	    fx25flag = 0;
	}

	if ((tag_no = fx25_search_tag(&fx25tag, bit)) > 0) { // correlation tag found

	  fx25tag = 0;
	  rs_code_size = tags[tag_no].rs_code;	// FX25 packet size

	  if (fx25_buf_size < rs_code_size) {	// buffer too small

	   printf("fx25_decode_bit(): rs_code_size = %d, fx25_buf_size = %d\n", rs_code_size, fx25_buf_size);

	    return -1; // buffer too small
	  }

	  state = STATE_DATA;

	  bzero(fx25_buf, rs_code_size);	// clear buffer
	  codeblock_bits = rs_code_size * 8;	// RS CODEBLOCK bit length

	  //printf("fx25_decode_bit(): tag: bit_offset = %d\n", bit_offset);
	  bit_offset = 0;
	  tag_t *tp = &tags[tag_no];
	  //printf("fx25_decode_bit(): found fx25 tag: %02x, (%d, %d), %d\n", tag_no, tags[tag_no].rs_code, tags[tag_no].rs_info, codeblock_bits);
	}

	break;

      case STATE_DATA:

	if (bit) fx25_buf[bit_offset / 8] |= 1 << (bit_offset % 8); // insert "1" to buffer
	bit_offset++;

	if (bit_offset >= codeblock_bits) {	// read all FX25 packet bits

	  //printf("fx25_decode_bit(): get packet: tag_no = %d, bit_offset = %d\n", tag_no, bit_offset);

  	  level_prev = 1;
	  state = STATE_SEARCH_TAG;
	  return tag_no;
	}
      }

      if (flag_found) return -2; // flag found

      return 0;
}

#define RS_CODE_SIZE 255
#define RS_INFO_SIZE 239

int fx25_rsdecode(uint8_t fx25_buf[], int tag_no)
{
    int i, j;
    int rs_code_size;
    int rs_info_size;
    int offset;
    int rs_parity;
    int rs_result;
    int factor;
    int rs_status;
    static uint8_t rs_buf[RS_CODE_SIZE];

    if ((tag_no >= CO_TAG_01) && (tag_no <= CO_TAG_0B)) { // FX.25 ver. 1.0 specification

	rs_code_size = tags[tag_no].rs_code;
	rs_info_size = tags[tag_no].rs_info;
	rs_parity = rs_code_size - rs_info_size;
	offset = RS_CODE_SIZE - rs_code_size;

	//if (offset > 0) bzero(rs_buf, offset);	// for shortend code, zero clear
	bzero(rs_buf, RS_CODE_SIZE);	// for shortend code, zero clear

	rs_buf[offset] = AX25_FLAG;		// first byte is always AX.25 flag (7E)

	for (i = 1; i < rs_code_size; i++) {	// copy data to RS buffer
	    rs_buf[offset + i] = fx25_buf[i];
	}

	rs_result = rs_decode(rs_buf, RS_CODE_SIZE, RS_CODE_SIZE - rs_parity);

	if (rs_result < 0) return rs_result; // RS decode fail

	//for (i = 0; i < rs_info_size; i++) {
	for (i = 0; i < rs_code_size; i++) {
	    fx25_buf[i] = rs_buf[offset + i];
	}
#if 0
	printf("fx25_rsdecode: rs_buf[]\n");
	for (i = 0; i < RS_CODE_SIZE; i++) {
		printf("%02x ", rs_buf[i]);
	}
	printf("\n");
#endif

	return rs_result;
    }

    if ((tag_no >= CO_TAG_0C) && (tag_no <= CO_TAG_0F)) { // Nemoto eXtension (NX)
	
	factor = tags[tag_no].rs_code / RS_CODE_SIZE;
	rs_code_size = RS_CODE_SIZE;
	rs_info_size = RS_INFO_SIZE;
	rs_parity = rs_code_size - rs_info_size;
	
	fx25_buf[0] = AX25_FLAG;			// first byte is always AX.25 flag (7E)

	rs_status = 0;					// sum of corrected symbols
	for (j = 0; j < factor; j++) {
	
	    for (i = 0; i < rs_code_size; i++) {
		rs_buf[i] = fx25_buf[j + i * factor];	// copy interleaved data to RS buffer
	    }

	    rs_result = rs_decode(rs_buf, RS_CODE_SIZE, RS_CODE_SIZE - rs_parity);

	    if (rs_result < 0) return rs_result; // fail

	    rs_status += rs_result;

	    for (i = 0; i < rs_code_size; i++) {
		fx25_buf[j + i * factor] = rs_buf[i];
	    }
	}

	return rs_status;
    }

    return -1; // unknown tag_no
}

#if 0
/*
 * RS error correction for FX25 packet
 */
int fx25_rsdecode(uint8_t buf[], int buf_size)
{
	  get_packet = 1;

	  // send fx.25 packet to next process
	  //afio_write(ofp, buf, bit_offset / 8, 0);
	  printf("get fx25 packet: bit length: %d\n", bit_offset);
#if 0
	  for (int j = 0; j < bit_offset/8; j++) {
	    printf("%02x ", buf[j]);
	  }
	  printf("\n");
	  int j;
	  for (j = 0; j < bit_offset/8; j++) {
	    int c = buf[j];
	    printf("%c", (c >= ' ' && c <= '~') ? c : '.');
	  }
	  printf("\n");
#endif

	  int rs_decode_done = 0;
	  static int rs_success = 0;
	  static int rs_decode_cnt = 0;

	  do {

	    ax25_len = bitstuff_decode(ax25_buf, ax25_buf_size, &buf[1], bit_offset/8 - 1); // skip AX25 flag
	    printf("ax25_len = %d\n", ax25_len);
	    if (ax25_len > 2) {
	      int fcs = ax25_fcs(ax25_buf, ax25_len - 2);
	      int fcs2 = ax25_buf[ax25_len - 2] | (ax25_buf[ax25_len - 1] << 8);

	      fprintf(stderr, "FCS: %04x\n", fcs);
#if 0
	      for (int j = 0; j < ax25_len; j++) {
	        printf("%02x ", ax25_buf[j]);
	      }
	      printf("\n");
#endif
	      if (fcs == fcs2) {
	        printf("FCS is correct\n");
		fcs_ok = 1;
		if (rs_decode_done) {
		  rs_success++;
		  printf("rs_decode success: %d / %d\n", rs_success, rs_decode_cnt);
		}

		printf("FX25 correct data: rs_decode = %d\n", rs_decode_cnt);
		for (int j = 0; j < ax25_len - 2; j++) {
		  int c = ax25_buf[j];

		  if (j < 14) c >>= 1;
		  printf("%c", isprint(c) ? c : '.');
		}
		printf("\n");

		break;

	      } else {
	        printf("FCS error\n");
	      }

	    } else {
	      printf("un-bit stuffing error\n");
	    }

	    if (!fcs_ok) {
	      uint8_t rs_buf[RS_CODE_SIZE];
	      int rs_code_size = tags[tag_no].rs_code;
	      int rs_info_size = tags[tag_no].rs_info;
	      int offset = RS_CODE_SIZE - rs_code_size;
	      int parity = rs_code_size - rs_info_size;
	      int rs_err = 0;

	      // RS decode
	      if (rs_decode_done) break;

	      static uint8_t pre_rs_decode[BUF_SIZE];

	      buf[0] = AX25_FLAG; // fixed value, avoid error correction
	      memcpy(pre_rs_decode, buf, rs_code_size);

	      if (tag_no <= CO_TAG_0B) { // FX.25 defined TAG

	      // buffer clear
	      bzero(rs_buf, RS_CODE_SIZE);

	      // copy RS code block
	      memcpy(rs_buf + offset, buf, rs_code_size);

	      
	      rs_err = rs_decode(rs_buf, RS_CODE_SIZE, RS_CODE_SIZE - parity);


	      // copy rs decode code
	      memcpy(buf, rs_buf + offset, rs_info_size);

	      } else if ((tag_no >= CO_TAG_0C) && (tag_no <= CO_TAG_0F)) {
		// Nemoto extension

	        // copy RS code block
	        buf[0] = AX25_FLAG; // fixed value, avoid error correction

		int m =	rs_code_size / RS_CODE_SIZE;
		int rs_sum = 0;
		for (int j = 0; j < m; j++) {
		  for (int i = 0; i < RS_CODE_SIZE; i++) {
		     rs_buf[i] = buf[i * m + j];
		  }

#define RS_INFO_SIZE 239

	          rs_err = rs_decode(rs_buf, RS_CODE_SIZE, RS_INFO_SIZE);

		  if (rs_err > 0) { // decode success
		     rs_sum += rs_err;

		     // copy RS decode data
		     for (int i = 0; i < RS_INFO_SIZE; i++) {
		       buf[i * m + j] = rs_buf[i];
		     }

		  } else if (rs_err < 0) { // decode error
		    rs_sum = rs_err;
		    break;
		  }
		}

		rs_err = rs_sum;
		offset = 0;
	      }

	      if (rs_status) *rs_status = rs_err;
	      rs_decode_done = 1;
	      rs_decode_cnt++;
	      fprintf(stderr, "rs_decode: %d\n", rs_err);

	      if (rs_err < 0) {
	        fprintf(stderr, "RS decode error: %d\n", rs_err);
	        break;
	      }

	      printf("RS decode result: %d\n", rs_err);
	      for (int i = 0; i < rs_info_size; i++) {
	        int c = pre_rs_decode[i];
	        int d = buf[i];

	        if (c == d) printf("%02x ", c);
	        else printf("(%02x->%02x) ", c, d);
	      }
	      printf("\n");
	    }
	  	
	  } while (!fcs_ok);

	  state = STATE_SEARCH_TAG;
	  fx25tag = 0;
	}
      }

    level = 1;
  }

  if (fcs_ok) return ax25_len;
  if (get_packet) return -1; // discard data due to error

  return 0;
}
#endif
