#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"

#include "fx25_framesync.h"
#include "fx25tag.h"
#include "config.h"

#ifdef CONFIG_TNC_DEMO_MODE
#include "freertos/freertos.h"
#include "freertos/ringbuf.h"
#include "uart.h"
#endif


#define FX25_CORRELATION_CNT 8

#define TAG "fx25_framesync"

#ifdef CONFIG_TNC_DEMO_MODE
int tag_error_pkts = 0;
#endif

static inline int bit_count(uint64_t bits)
{
#ifdef __XTENSA__
    uint32_t t0 = (uint32_t)bits;
    uint32_t t1 = (uint32_t)(bits >> 32);
    uint32_t t2, t3;
    uint32_t c55 = 0x55555555;
    uint32_t c33 = 0x33333333;
    uint32_t c0f = 0x0f0f0f0f;

    asm volatile(
	    // x = x - ((x >> 1) & 0x55555555);
	    // low 32bit
	    "srli	%2, %0, 1\n\t"	// x >> 1
	    "and	%2, %2, %4\n\t" // (x >> 1) & 0x55555555
	    "sub	%0, %0, %2\n\t"	// x - ((x >> 1) & 0x55555555)

	    // high 32bit
	    "srli	%2, %1, 1\n\t"	// x >> 1
	    "and	%2, %2, %4\n\t" // (x >> 1) & 0x55555555
	    "sub	%1, %1, %2\n\t"	// x - ((x >> 1) & 0x55555555)

	    // x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
	    // low 32bit
	    "and	%2, %0, %5\n\t" // (x & 0x33333333)
	    "srli	%3, %0, 2\n\t"	// x >> 2
	    "and	%3, %3, %5\n\t"	// (x >> 2) & 0x33333333;
	    "add.n	%0, %2, %3\n\t"	// (x & 0x33333333) + ((x >> 2) & 0x33333333)

	    // high 32bit
	    "and	%2, %1, %5\n\t" // (x & 0x33333333)
	    "srli	%3, %1, 2\n\t"	// x >> 2
	    "and	%3, %3, %5\n\t"	// (x >> 2) & 0x33333333;
	    "add.n	%1, %2, %3\n\t"	// (x & 0x33333333) + ((x >> 2) & 0x33333333)

	    // x = (x + (x >> 4)) & 0x0f0f0f0f
	    // low 32bit
	    "srli	%2, %0, 4\n\t"	// x >> 4
	    "add.n	%0, %0, %2\n\t"	// x + (x >> 4)
	    "and	%0, %0, %6\n\t"	// (x + (x >> 4)) & 0x0f0f0f0f

	    // high 32bit
	    "srli	%2, %1, 4\n\t"	// x >> 4
	    "add.n	%1, %1, %2\n\t"	// x + (x >> 4)
	    "and	%1, %1, %6\n\t"	// (x + (x >> 4)) & 0x0f0f0f0f

	    // x = x + (x >> 32)
	    "add.n	%0, %0, %1\n\t"	// low 32bit + high 32bit

	    // x = x + (x >> 8)
	    "srli	%2, %0, 8\n\t"	// x >> 8
	    "add.n	%0, %0, %2\n\t"	// x + (x >> 8)

	    // x = x + (x >> 16)
	    "srli	%2, %0, 16\n\t"	// x >> 16
	    "add.n	%0, %0, %2\n\t"	// x + (x >> 16)

	    // x & 0x7f
	    "extui	%0, %0, 0, 7\n\t" // x = x & 0x7f

	    :
	    "+r"(t0),			// 0
	    "+r"(t1),			// 1
	    "=&r"(t2),			// 2
	    "=&r"(t3)			// 3
	    :
	    "r"(c55),			// 4
	    "r"(c33),			// 5
	    "r"(c0f)			// 6
	    :
       );

    return t0;
#else
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
#endif
}

int fx25_search_tag(uint64_t *correlation_tag, int data_bit)
{
	int count;
	int i;
	uint64_t bits;

	/* add 1 bit to tag data */
	*correlation_tag >>= 1;
	*correlation_tag |= ((uint64_t)data_bit << 63);

	/* compare tag with TAG_01 to TAG_1F */
	for (i = CO_TAG_01; i <= CO_TAG_1F; i++) {

	    bits = *correlation_tag ^ tags[i].tag;
 		count = bits ? bit_count(bits) : 0; // set bits if different 

#ifdef CONFIG_TNC_DEMO_MODE 
 	   if (count > 0 && count <= 12) { // 12bit will cause count miss rate about 1e-3
#define INFO_BUF_SIZE 80
	    	char buf[INFO_BUF_SIZE];
    		int len;

    		len = snprintf(buf, INFO_BUF_SIZE, "\tFX25 info: Tag error %d bits, bit pattern: %016llx\n", count, bits);
    		packet_output((uint8_t *)buf, len);
    		if (count > FX25_CORRELATION_CNT) tag_error_pkts++;
#if 0
    		fprintf(stderr, "fx25_decode: correlation tag match %d bits\n", 64 - count);
    		fprintf(stderr, "bit_count(%016llx) = %d\n", bits, count);
    		fprintf(stderr, "%016llx\n%016llx\n", *correlation_tag, tags[i].tag);
#endif
    	}
#endif

    	if (count <= FX25_CORRELATION_CNT) {

    		if (count != 0) {
    			ESP_LOGI(TAG, "bit_count(%016llx) = %d", bits, count);
    			ESP_LOGI(TAG, "%016llx\n%016llx\n", *correlation_tag, tags[i].tag);
      		}

    		return i; // find i-th TAG
    	}	

	}

	return -1; // not found tag
}

int fx25_match_2nd_tag(uint64_t *correlation_tag, int data_bit)
{
	static int code_tag_readlen = 0;
	int count;
	int i;
	uint64_t bits;

	/* add 1 bit to tag data */
	*correlation_tag >>= 1;
	*correlation_tag |= ((uint64_t)data_bit << 63);
	code_tag_readlen++;
	if (code_tag_readlen < 63) return -1; // continue code tag

	code_tag_readlen = 0;

	/* compare code tag with CODE_TAG_01 to CODE_TAG_03 */
	for (i = CODE_TAG_01; i <= CODE_TAG_03; i++) {

	    bits = *correlation_tag ^ code_tags[i].tag;
 		count = bits ? bit_count(bits) : 0; // set bits if different 

    	if (count <= FX25_CORRELATION_CNT) {
    		if (count != 0) {
    			ESP_LOGI(TAG, "bit_count(%016llx) = %d", bits, count);
    			ESP_LOGI(TAG, "%016llx\n%016llx\n", *correlation_tag, code_tags[i].tag);
      		}

    		return i; // find i-th CODE TAG
    	}	

	}

	return -2; // no match code tag
}
