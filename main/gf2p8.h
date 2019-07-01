/*
  Operations on Galois Field GF(2^8)
*/

#ifndef GF2P8_H

#include <stdio.h>
#include <stdlib.h>

#define GF2P8_ELEMENT 256
#define GF2P8_ORDER 255

typedef unsigned char gf2p8_t;

/*
  initialize internal tables
*/
void gf2p8_init(void);


/*
  addtion
  return x + y
*/
gf2p8_t gf2p8_add(gf2p8_t x, gf2p8_t y);

/*
  subtraction
  return x - y
*/
gf2p8_t gf2p8_sub(gf2p8_t x, gf2p8_t y);



/*
  index
  return y where a^y = x
 */
gf2p8_t gf2p8_ind(gf2p8_t x);

/*
  power
  return a^x
 */
gf2p8_t gf2p8_pow(gf2p8_t x);

/*
  multiplication
  return x * y
*/
gf2p8_t gf2p8_mul(gf2p8_t x, gf2p8_t y);


/*
  reciplocal number
  return y where x * y = 1
 */
gf2p8_t gf2p8_recip(gf2p8_t x);


/*
  dividing
  return x / y
*/
gf2p8_t gf2p8_div(gf2p8_t x, gf2p8_t y);


/*
  negate
  return -x
*/
gf2p8_t gf2p8_neg(gf2p8_t x);

#define GF2P8_H 1
#endif
