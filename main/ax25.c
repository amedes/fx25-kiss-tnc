/*
 * AX.25 related routine
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <rom/crc.h>

#include "ax25.h"

#define CRC16_POLY 0x10811 /* G(x) = 1 + x^5 + x^12 + x^16 */

/*
 * calculate AX.25 FCS (CRC16)
 */
int ax25_fcs(uint8_t packet[], int length)
{
#if 0
    uint32_t crc;
    int i, j;

    if (length <= 0) return -1; // packet too short

    // calculate CRC x^16 + x^12 + x^5 + 1
    crc = 0xffff; /* initial value */
    for (i = 0; i < length; i++) {
	crc ^= packet[i];
	for (j = 0; j < 8; j++) {
	    if (crc & 1) crc ^= CRC16_POLY;
	    crc >>= 1;
	}
    }
    crc ^= 0xffff; // invert

    return crc;
#else
    return crc16_le(0, packet, length); // ESP32 ROM routine, faster than my code :-)
#endif
}

/*
 * check FCS (last 2 byte)
 *
 * return true if OK
 */
int ax25_fcs_check(uint8_t buf[], int len)
{
    int fcs;
    int fcs2;

    if (len <= 2) return 0;

    fcs = ax25_fcs(buf, len - 2);

    if (fcs < 0) return 0;

    fcs2 = buf[len - 2] | (buf[len - 1] << 8);

    return fcs == fcs2;
}

/*
 * count bit length of the AX.25 packet with bit stuffing
 */
int ax25_count_bit_length(uint8_t packet[], int length)
{
  int bit_offset = 0;
  int bit_length;
  int data_bit;
  int bit1_len;

  bit_length = 0;
  bit1_len = 0;
  for (bit_offset = 0; bit_offset < length * 8; bit_offset++) {
    data_bit = (packet[bit_offset / 8] >> bit_offset % 8) & 1;
    bit_length++;

    /* count countinuous '1' */
    if (data_bit) { // '1'
      if (++bit1_len >= 5) { // need bit stuffing
	bit_length++;
	bit1_len = 0;
      }
    } else { // '0'
      bit1_len = 0;
    }
  }

  return bit_length;
}

#define AX25_ADDR_LEN 7
#define AX25_ADDR_SSID 6
/*
 * convert ASCII callsign to AX.25 address with SSID
 */
char *ax25_call_to_addr(const char *callsign)
{
    static char ax25_addr[AX25_ADDR_LEN];
    int i;
    int ssid;

    if (callsign == NULL) return NULL;

    memset(ax25_addr, ' ' << 1, AX25_ADDR_LEN - 1);
    ax25_addr[AX25_ADDR_SSID] = 0x60; // SSID 0

    for (i = 0; i < AX25_ADDR_SSID; i++) {
	char c = toupper((int)callsign[i]);

	if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z')) {
	    ax25_addr[i] = c << 1;
	} else {
	    break;
	}
    }

    // SSID
    if (callsign[i] == '-') {
	int c = callsign[++i];

	if (c >= '1' && c <= '9') {
	    ssid = c - '0';
	    if (ssid == 1) {
		c = callsign[++i];
		if (c >= '0' && c <= '5') {
		    ssid *= 10;
		    ssid += c - '0';
		}
	    }
	    ax25_addr[AX25_ADDR_SSID] |= ssid << 1;
	}
    }

    return ax25_addr;
}
