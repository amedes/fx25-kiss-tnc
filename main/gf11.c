/*
  Operations on Galois Field GF(11)
*/

#include "gf11.h"

#define GF_PRIME 11

static gf11_t gf11_index[GF11_ELEMENT];
static gf11_t gf11_power[GF11_ELEMENT];
static gf11_t gf11_reciprocal[GF11_ELEMENT];

/*
  initialize internal tables
*/
void gf11_init(void)
{
  static int init_flag = 0;
  int i, j;
  gf11_t a = 2; /* primitive element */
  int n = 1; /* a^0 */

  if (init_flag) return;

  /* make index and power tables */
  for (i = 0; i < GF11_ELEMENT - 1; i++) {
    gf11_index[n] = i; /* n = a^i */
    gf11_power[i] = n; /* a^i = n */

    n = gf11_mul(n, a); /* a^(i+1) = a^n * a, increment index of a^n */
  }
  gf11_index[0] = GF_PRIME - 1; // 0 has no index
  gf11_power[GF_PRIME - 1] = 1; // a^(GF_PRIME - 1) = 1

  /* make reciplocal number table */
  for (i = 1; i < GF11_PRIME; i++) {
    for (j = 1; j < GF11_PRIME; j++) {
      if (gf11_mul(i, j) == 1) {
	gf11_reciprocal[i] = j;
	break;
      }
    }
  }
  gf11_reciprocal[0] = 0; // 0 has no reciprocal number;

  init_flag = 1;
}

/*
  addtion
  return x + y
*/
gf11_t gf11_add(gf11_t x, gf11_t y)
{
  return (x + y) % GF_PRIME;
}

/*
  subtraction
  return x - y
*/
gf11_t gf11_sub(gf11_t x, gf11_t y)
{
  return (x >= y) ? (x - y) : (x + GF_PRIME - y);
}


/*
  index
  return y where a^y = x
 */
gf11_t gf11_ind(gf11_t x)
{
  return gf11_index[x];
}

/*
  power
  return a^x
 */
gf11_t gf11_pow(gf11_t x)
{
  return gf11_power[x];
}

/*
  multiplication
  return x * y
*/
gf11_t gf11_mul(gf11_t x, gf11_t y)
{
  return (x * y) % GF_PRIME;
}

/*
  reciplocal number
  return y where x * y = 1
 */
gf11_t gf11_recip(gf11_t x)
{
  if (x <= 1) return x; // return 0 if x == 0

  return gf11_reciprocal[x];
}

/*
  dividing
  return x / y
*/
gf11_t gf11_div(gf11_t x, gf11_t y)
{
  return gf11_mul(x, gf11_recip(y));
}

/*
  negate
  return -x
*/
gf11_t gf11_neg(gf11_t x)
{
  if (x == 0) return 0;

  return GF_PRIME - x;
}
