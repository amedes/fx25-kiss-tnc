/*
 * ax25_decode.c
 *
 * decode AX.25 packet
 */

#include <stdio.h>
#include <stdint.h>
#include <strings.h>

#include "ax25.h"

#define AX25_FLAG 0x7e
#define BUF_SIZE 1024
#define MAX_BITS_LEN 7 // max bits length in AX.25 NRZI packet
#define BIT_STUFF_LEN 5 // need bit stuffing "1"
#define FLAG_BIT_LEN 7 // flag NRZI length

void print_bits(uint8_t buf[], int len)
{
	printf("nrzi bits: len = %d\n", len);
#if 0
	for (int i = 0; i < len; i++)
		printf("%d ", buf[i]);
	printf("\n");
#endif
}

/*
 * extract AX.25 NRZI data and store memory
 */
int ax25_nrzi_bit(int level, uint8_t buf[], int len)
{
  static int level_prev = 1;
  static uint8_t flag = 0;
  static uint16_t byte = 0xff00;
  static int index = 0;
  static int in_packet = 0;
  static int invert = 0;
  int bit;

  if (level < 0) { // initialize internal variables
    level_prev = 1;
    flag = 0;
    byte = 0xff00;
    index = 0;
    in_packet = 0;

    return 0;
  }

  // NRZI
  bit = !(level ^ level_prev);
  level_prev = level;

  flag >>= 1;
  if (bit) flag |= 0x80;

  if (flag == AX25_FLAG) { // found flag
    int idx = index;
    uint16_t bt = byte;
    int cnt;

//    printf("ax25_nrzi_bit(): flag found, byte = %04x, index = %d, in_packet = %d\n", byte, index, in_packet);

    invert = level; // adjust polarity
    flag = 0;
    byte = 0xff00;
    index = 0;

    if (!in_packet) {

      in_packet = 1;
      return -2;	// flag found
    }

    if (idx == 0) return -2; // flag found

    // shift until data reach LSb
    cnt = 0;
    while (bt & 0xff00) {
      bt >>= 1;
      cnt++;
    }

    return idx * 8 - (cnt - 1);
  }

  if (!in_packet) return 0;

  // do not NRZI
  byte >>= 1;
  if (level ^ invert) byte &= ~0x80; // clear MSb

  if (byte & 0xff00) return 0; // not byte boundary

  if (index < len) {
    buf[index++] = byte;
    byte = 0xff00;

    return 0;
  }

  // buffer overflow
  //

  flag = 0;
  byte = 0xff00;
  index = 0;
  in_packet = 0;

  return -1;
}

/*
 * ax25_nrzi: find AX.25 flag (7E) and NRZI pulses store memory
 *
 * return number of NRZI bits if find flag else return 0
 */
int ax25_nrzi(int bits, uint8_t buf[], int len)
{
  static int level = 1; // always start "1" pulse
  static int offset = 0;
  //static uint8_t flag = 0;
  //int bit;
  static int del_next_1bit = 0;
  static int in_packet = 0;
#if 0
  static uint8_t nrzi[256];
  static int idx = 0;

  nrzi[idx++] = bits;
#endif

  // check error
  if ((bits <= 0) || (bits > FLAG_BIT_LEN)) {

    printf("ax25_nrzi: wrong bits = %d, offset = %d\n", bits, offset);

    level = 1;
    offset = 0;
    //flag = 0;
    del_next_1bit = 0;
    in_packet = 0;
#if 0
    print_bits(nrzi, idx);
    idx = 0;
#endif

    return 0;
  }

  // check flag
  if (bits == FLAG_BIT_LEN) {
    int bit_len = offset;
    int in = in_packet;

    level = 1;
    offset = 0;
    del_next_1bit = 1;
    in_packet = 1;

    if (in) return bit_len;

    return 0;
  }

  if (!in_packet) return 0;

  //bit = 0; // un-NRZI bit value

  if (del_next_1bit) {
    --bits;
    //bit = 1;
    del_next_1bit = 0;
  }

  while (bits-- > 0) {
    
      if (buf && (offset < len * 8)) {
        if (level)
   	  buf[offset / 8] |= 1 << (offset % 8);
        else
	  buf[offset / 8] &= ~(1 << (offset % 8));
      }

      offset++;
      //bit = 1;

  }
  level = !level; // invert

  return 0;
}

/*
 * decode AX.25 NRZI signal
 */
int ax25_decode(int bits, uint8_t buf[], int len)
{
  int level;
  static uint8_t flag = 0xff;
  static int bit_offset = 0;
  static int del_next_zero = 0;
  static int bit1 = 0;
  int buf_bit_size = len * 8;
  //static int del_next_1bit = 0;

  if ((bits <= 0) || (bits > MAX_BITS_LEN)) {
     flag = 0xff;
     bit_offset = 0;
     del_next_zero = 0;
     bit1 = 0;

     return 0;
  }

  level = 0;

  while (bits-- > 0) {

    flag >>= 1;
    if (level) flag |= 0x80; // insert bit at MSb

    if (flag == AX25_FLAG) {
      int size = bit_offset / 8;
      int offs = bit_offset % 8;

      flag = 0xff;
      bit_offset = 0;
      del_next_zero = 0;
      bit1 = 0;
      
      if ((offs % 8) == 7) { // found AX.25 Packet
        return (size < len) ? size : len; 
      } else { // may be noise...
	return 0;
      }
    }

    if (level) { // value "1"
      if (buf && (bit_offset < buf_bit_size)) {
	buf[bit_offset / 8] |= (1 << bit_offset % 8);
      }
      bit_offset++;

      bit1++;
      if (bit1 >= BIT_STUFF_LEN) {
        del_next_zero = 1;
	bit1 = 0;
      }
    } else { // value "0"

      if (!del_next_zero) {

	// insert "0"
        if (buf && (bit_offset < buf_bit_size)) {
	  buf[bit_offset / 8] &= ~(1 << bit_offset % 8);
        }
	bit_offset++;
      }
      bit1 = 0;
      del_next_zero = 0;
    }
    level = 1;
  }

  return 0;
}

/*
 * decode AX.25 bit sequence
 *
 * return:
 * 	> 0 found packet
 * 	0   store data in progress
 * 	< 0 error
 */
int ax25_decode_bit(int bit, uint8_t buf[], int len)
{
  static uint8_t flag = 0x00;
  static uint16_t byte = 0xff00;
  static int index = 0;
  static int in_packet = 0;

  if (bit < 0) {

    // reset internal variables
    flag = 0x00;
    byte = 0xff00;
    index = 0;
    in_packet = 0;

    return 0;
  }

  // flag detection
  flag >>= 1;		// LSb first
  if (bit) flag |= 0x80; // insert bit to MSb
  
  if (flag == AX25_FLAG) { // found flag
    int byte_boundary = byte == 0x01fc; // always this value, if byte boundary
    int idx = index;

#if 0
    printf("ax25_decode_bit(): found flag: index = %d, byte = %04x, in_packet = %d, byte_boundary = %d, buf[%d] = %02x\n", index, byte, in_packet, byte_boundary,
		   (index-1 < 0) ? 0 : index-1,
		   buf[(index-1 < 0) ? 0 : index-1]);
    for (int i = 0; i < index; i++) printf("%02x ", buf[i]);
    printf("\n");
#endif

    flag = 0x00;
    byte = 0xff00;
    index = 0;

    if (!in_packet) {
      in_packet = 1;

      return 0;
    }
    
    if (byte_boundary) return idx;

    // not byte boundary
    return -1;
  }

  // not in packet
  if (!in_packet) return 0;

  // bit stuffing detection
  if ((flag & 0xfe) == 0x7c) { // (MSb) 0111110XX (LSb) delete MSb "0"
	  // do nothing (delete "0")
  } else {

    byte >>= 1; // insert "1" to MSb

    if (!bit) { // "0"
      byte &= ~0x80; // clear MSb
    }
  }

  if (byte & 0xff00) return 0; // not enough for byte

  if (index < len) {

    buf[index++] = byte;
    byte = 0xff00; // reset byte buffer

    return 0;
  }

  // buffer overflow
  flag = 0x00;
  byte = 0xff00;
  index = 0;
  in_packet = 0;

  return -1;
}

/*
 * decode NRZI and bit stuffing
 */
int nrzi_decode(uint8_t buf[], int len, uint8_t nrzi[], int nrzi_bitlen)
{
  int nrzi_off = 0;
  int bit_offset = 0;
  int level;
  int lprev = 1;
  int del_next_zero = 0;
  int bit1 = 0;

  // clear buffer
  bzero(buf, len);

  while (nrzi_off < nrzi_bitlen) {
    
    level = (nrzi[nrzi_off / 8] & (1 << (nrzi_off % 8))) != 0;
    nrzi_off++;

    if (level == lprev) { // mean "1" 

      if (del_next_zero) { // may be error
	return 0;
      }

      // insert "1"
      if (buf && (bit_offset < len * 8)) buf[bit_offset / 8] |= 1 << (bit_offset % 8);
      bit_offset++;

      bit1++;
      if (bit1 >= BIT_STUFF_LEN) {
	del_next_zero = 1;
	bit1 = 0;
      }

    } else { // "0"

      if (del_next_zero) {
        del_next_zero = 0;
      } else {
	bit_offset++; // insert "0"
      }

      bit1 = 0;

    }
    lprev = level;
  }

  return bit_offset;
}

#define AX25_MIN_LEN (7 * 2 + 1 + 1+ 2)

/*
 * Backward Error Correction
 */
int ax25_bec(uint8_t ax25_buf[], int ax25_len, uint8_t nrzi_buf[], int nrzi_len)
{
    int bit_size;
    int byte_size;
    int i;

    for (i = 0; i < nrzi_len; i++) {
	nrzi_buf[i >> 3] ^= 1 << (i & 7); // invert bit

	bit_size = nrzi_decode(ax25_buf, ax25_len, nrzi_buf, nrzi_len);

	if ((bit_size & 7) == 0) {
	    
	    byte_size = bit_size >> 3;
	    if (byte_size >= AX25_MIN_LEN) {

		if (ax25_fcs_check(ax25_buf, byte_size)) {
		    return byte_size;
		}
	    }
	}

	nrzi_buf[i >> 3] ^= 1 << (i & 7); // restore bit
    }

    return 0;
}
