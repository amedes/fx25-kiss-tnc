/*
  Galoris Feild processing routines
*/

#ifndef GF_H

/* uncomment one of these identfiers */

#define USE_GF2P8 1
//#define USE_GF11 1

#ifdef USE_GF2P8

#include "gf2p8.h"

#define GF_ORDER GF2P8_ORDER

#define gf_t gf2p8_t

#define gf_init gf2p8_init
#define gf_add gf2p8_add
#define gf_sub gf2p8_sub
#define gf_mul gf2p8_mul
#define gf_div gf2p8_div
#define gf_pow gf2p8_pow
#define gf_ind gf2p8_ind
#define gf_neg gf2p8_neg

#endif

#ifdef USE_GF11

#include "gf11.h"

#define GF_ORDER GF11_ORDER

#define gf_t gf11_t

#define gf_init gf11_init
#define gf_add gf11_add
#define gf_sub gf11_sub
#define gf_mul gf11_mul
#define gf_div gf11_div
#define gf_pow gf11_pow
#define gf_ind gf11_ind
#define gf_neg gf11_neg

#endif

#define GF_H
#endif
