/*
 * bit_stuffing.c
 *
 * add and delete bit stuffing bit
 */

#include <stdio.h>
#include <stdint.h>
#include <strings.h>

#include "modem.h"

//#include "afio.h"


#define BUF_SIZE 256
#define ITEM32_SIZE (BUF_SIZE*8+1)
#define TXD_DURATION (1000*1000 / BAUD_RATE) // 1/1200 usec

#define BIT_STUFF_LEN 5
#define BITST_SIZE (BUF_SIZE + (((BUF_SIZE * 8) / 5) + 7) / 8) // need 6/5 times

#define FEND 0xc0
#define FESC 0xdb
#define TFEND 0xdc
#define TFESC 0xdd

static int bitstuff_byte(uint8_t bit_buf[], int byte, int bt_flag, int buf_len)
{
  static int bit_offset;
  static int bit1_count;
  int i;
  int c;
  int bitbuf_len = buf_len * 8;

  // inilialize variables
  if (bit_buf == NULL) {
    bit_offset = 0;
    bit1_count = 0;

    return 0;
  }

  c = byte;
  for (i = 0; i < 8; i++) {

    if (c & 1) { // bit value "1"

      if (bit_buf && bit_offset < bitbuf_len)
	      bit_buf[bit_offset / 8] |= 1 << (bit_offset % 8); // set bit "1"

      if (bt_flag) { // do not need bit stuffing for AX25 flag

        bit1_count++; // count number of "1"
        if (bit1_count >= BIT_STUFF_LEN) { // need bit stuffing

          // insert "0"
	  bit_offset++;
          bit1_count = 0;
        }
      }

    } else { // bit value "0"
      bit1_count = 0; // clear counter
    } 
    c >>= 1; // shift next bit
    bit_offset++;

  }

  //fprintf(stderr, "%x, %d, %d\n", byte, bt_flag, bit_offset);

  return bit_offset;
}

#define AX25_FLAG 0x7e

/*
 * encode bit stuffing
 */
int bitstuff_encode(uint8_t bits_buf[], int bits_len, uint8_t buf[], int len)
{
  int byte_offset;
  int bit_offset;
  int i;
  int bit_size;

  // clear buffer
  bzero(bits_buf, bits_len);

  // initialize internal variables
  bitstuff_byte(NULL, 0, 0, 0);

  // packet start flag
  bit_offset = bitstuff_byte(bits_buf, AX25_FLAG, 0, bits_len); // do not bit stuffing

  // add each byte to buffer with bit stuffing
  for (i = 0; i < len; i++) {
    bit_offset = bitstuff_byte(bits_buf, buf[i], 1, bits_len);
  }

  // packet end flag
  bit_offset = bitstuff_byte(bits_buf, AX25_FLAG, 0, bits_len); // do not bit stuffing

  // padding
  bit_size = bit_offset % 8;
  if (bit_size > 0) {
    byte_offset = bit_offset / 8;
    if (byte_offset < bits_len) {
      bits_buf[byte_offset] |= AX25_FLAG & ~((1 << bit_size) - 1);
    }
    bit_offset += 8 - bit_size;
  }

  //afio_write(fp, nrzi, byte_size, padding);

  return bit_offset / 8; // return byte length
}

/*
 * decode bit stuffing
 */
int bitstuff_decode(uint8_t byte_buf[], int buf_len, uint8_t ax25_buf[], int bits_len)
{
  int bit_offset;
  int bit_offset_len = bits_len * 8;
  int byte = 0;
  int bit;
  int i, j;
  int bit1 = 0;
  int delete_next_zero = 0;
  int byte_bit_offset = 0;
  int byte_bit_offset_len = buf_len * 8;

  // clear buffer
  bzero(byte_buf, buf_len);

  bit_offset = 0;
  while (bit_offset < bit_offset_len) {
    bit = (ax25_buf[bit_offset / 8] >> (bit_offset % 8)) & 1;
    bit_offset++;

    if (bit) { // "1"

      // find AX.25 end flag?
      if (delete_next_zero) { // find 6 consecutive "1"
	if (byte_bit_offset % 8 == 6) { // make sure bit position
	  return byte_bit_offset / 8;   // return number of bytes
	} else {
	  return -2; // error
	}
      }

      if (byte_bit_offset >= byte_bit_offset_len) return -3; // buffer overflow

      byte_buf[byte_bit_offset / 8] |= 1 << (byte_bit_offset % 8);
      byte_bit_offset++;

      bit1++; // count "1"
      if (bit1 >= BIT_STUFF_LEN) {
	delete_next_zero = 1;
	bit1 = 0;
      }

    } else { // "0"

      bit1 = 0;
      if (delete_next_zero) {
	// no insert 0
	delete_next_zero = 0;
      } else {
	byte_bit_offset++; // insert 0
      }
    }
  }

  return -1;
}

#define STATE_INFRAME 1
#define STATE_OUTFRAME 2
#define STATE_ESCAPE 3

#ifdef notdef
int main(int argc, char *argv[])
{
  FILE *ifp = stdin;
  FILE *ofp = stdout;
  char buf[BUF_SIZE];
  int c;
  int index;
  int state;
  int bit_length;

  while ((bit_length = afio_read(ifp, buf, BUF_SIZE)) >= 0) {
    nrzi_encode(ofp, buf, bit_length / 8);
  }
  return 0;
}
#endif

//#include "afio.h"

#define BIT_TIME (80*1000*1000 / BAUD_RATE) // 1 bit time in 80MHz ticks
#define BIT_FLAG_LEN 7 // flag 0x7e, continuos "1" bit length + 1
//#define BIT_STUFF_LEN 6 // 6 bits length followed by stuffing bit "0"

//#define BUF_SIZE 1024

void nrzi_packet(FILE *fp, uint8_t buf[], int bit_len)
{
  int nmemb = bit_len / 8;

  if (nmemb > 0) fwrite(buf, sizeof(uint8_t), nmemb, fp);
}

#define STATE_SEARCH_FLAG 1
#define STATE_DATA 2

#ifdef notdef
int main(int argc, char *argv[])
{
  FILE *ifp = stdin;
  FILE *ofp = stdout;
  uint8_t buf[BUF_SIZE];
  uint32_t ts, ts0 = 0;
  int level;
  int bit_offset;
  int bit_stuff;
  int delete_zero;
  int bit_len;
  int byte_size;
  int padding;
  int state;

  state = STATE_SEARCH_FLAG;
  delete_zero = 0;
  bit_stuff = 0;
  bit_offset = 0;
  bzero(buf, BUF_SIZE);

  while (fread(&ts, sizeof(ts), 1, ifp) == 1) {
    level = ts & 1;
    bit_len = (ts - ts0 + BIT_TIME/2) / BIT_TIME;
    ts0 = ts;

    switch (state) {
    case STATE_SEARCH_FLAG:
      if (bit_len == BIT_FLAG_LEN) {
	state = STATE_DATA;
	delete_zero = 1; // delete next "0"
	bit_stuff = 0;
	bit_offset = 0;
	bzero(buf, BUF_SIZE);
      }
      break;

    case STATE_DATA:
      if (bit_len >= BIT_FLAG_LEN) {
	byte_size = (bit_offset + 7) / 8;
	padding = byte_size * 8 - bit_offset;
	if (byte_size > 0) {
	  afio_write(ofp, buf, byte_size, padding);
	}

	bit_stuff = 0;
	bit_offset = 0;
	bzero(buf, BUF_SIZE);

	if (bit_len == BIT_FLAG_LEN) { // detect 7e
	  delete_zero = 1;
	} else {
	  delete_zero = 0;
	  state = STATE_SEARCH_FLAG;
	}
	continue;
      }
    
      if (bit_len > BIT_STUFF_LEN) {
	bit_stuff = 1;
      }

      // de-NRZI
      if (delete_zero) {
	delete_zero = 0; // do not add "0"
      } else {
	bit_offset++; // add "0"
      }

      while (--bit_len > 0) { // add "1"
	buf[bit_offset / 8] |= 1 << (bit_offset % 8);
	bit_offset++;
      }

      if (bit_stuff) {
	delete_zero = 1; // delete next "0"
	bit_stuff = 0;
      }
    }
  }

  return 0;
}
#endif
