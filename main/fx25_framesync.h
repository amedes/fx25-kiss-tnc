/*
 * fx25_framesync()
 * sync pn code on FX.25 packet.
 * 
 */
#ifndef __FRAMESYNC_H__
#define __FRAMESYNC_H__ 1

#include <stdint.h>

int fx25_search_tag(uint64_t *correlation_tag, int data_bit);
int fx25_match_2nd_tag(uint64_t *correlation_tag, int data_bit);

#endif