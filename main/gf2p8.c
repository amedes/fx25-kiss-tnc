/*
  Operations on Galois Field GF(2^8)
*/

#include "gf2p8.h"

static gf2p8_t gf2p8_index[GF2P8_ELEMENT];
static gf2p8_t gf2p8_power[GF2P8_ELEMENT];

/*
  initialize internal tables
*/
void gf2p8_init(void)
{
  int i;
  const int b = 0x1d; /* x^8 = x^4 + x^3 + x^2 + 1*/
  int n = 1; /* a^0 */

  /* make index and power tables */
  for (i = 0; i < GF2P8_ELEMENT - 1; i++) {
    gf2p8_index[n] = i; /* n = a^i */
    gf2p8_power[i] = n; /* a^i = n */

    n <<= 1; /* a^(i+1) = a^n * a, increment index of a^n */
    if (n > 255) {
      /* calculate bit vector notation */
      n = (n & 0xff) ^ b; /* a^n = (a^n - a^8) + (a^4 + a^3 + a^2 + 1) */
    }
  }
  gf2p8_index[0] = 255; // 0 has no index
  gf2p8_power[255] = 1; // a^255 = 1
}

/*
  addtion
  return x + y
*/
gf2p8_t gf2p8_add(gf2p8_t x, gf2p8_t y)
{
  return x ^ y;
}

/*
  subtraction
  return x - y
*/
gf2p8_t gf2p8_sub(gf2p8_t x, gf2p8_t y)
{
  return gf2p8_add(x, y); /* same as addtion */
}


/*
  index
  return y where a^y = x
 */
gf2p8_t gf2p8_ind(gf2p8_t x)
{
  return gf2p8_index[x];
}

/*
  power
  return a^x
 */
gf2p8_t gf2p8_pow(gf2p8_t x)
{
  return gf2p8_power[x];
}

/*
  multiplication
  return x * y
*/
gf2p8_t gf2p8_mul(gf2p8_t x, gf2p8_t y)
{
  int ind;

  if (x == 0 || y == 0) return 0;

  ind = gf2p8_ind(x) + gf2p8_ind(y);
  ind %= GF2P8_ORDER;

  return gf2p8_pow(ind);
}

/*
  reciplocal number
  return y where x * y = 1
 */
gf2p8_t gf2p8_recip(gf2p8_t x)
{
  if (x <= 1) return x; // return 0 if x == 0

  return gf2p8_pow(GF2P8_ORDER - gf2p8_ind(x));
}

/*
  dividing
  return x / y
*/
gf2p8_t gf2p8_div(gf2p8_t x, gf2p8_t y)
{
  return gf2p8_mul(x, gf2p8_recip(y));
}

/*
  negate
  return -x
*/
gf2p8_t gf2p8_neg(gf2p8_t x)
{
  return x; // x == -x on GF(2^n)
}
