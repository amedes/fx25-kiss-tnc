#include <stdio.h>
#include <stdint.h>
#include <strings.h>

#include "modem.h"
//#include "afio.h"

#define BIT_TIME (80*1000*1000 / BAUD_RATE) // 1 bit time in 80MHz ticks
#define BIT_FLAG_LEN 7 // flag 0x7e, continuos "1" bit length + 1
#define BIT_STUFF_LEN 6 // 6 bits length followed by stuffing bit "0"

#define BUF_SIZE 1024

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
    
      if (bit_len >= BIT_STUFF_LEN) {
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
