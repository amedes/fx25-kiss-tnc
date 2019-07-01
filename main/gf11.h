/*
  Operations on Galois Field GF(11)
*/

#ifndef GF11_H

#define GF11_ELEMENT 11
#define GF11_ORDER 10
#define GF11_PRIME 11

typedef unsigned char gf11_t;

/*
  initialize internal tables
*/
void gf11_init(void);


/*
  addtion
  return x + y
*/
gf11_t gf11_add(gf11_t x, gf11_t y);

/*
  subtraction
  return x - y
*/
gf11_t gf11_sub(gf11_t x, gf11_t y);

/*
  index
  return y where a^y = x
 */
gf11_t gf11_ind(gf11_t x);

/*
  power
  return a^x
 */
gf11_t gf11_pow(gf11_t x);

/*
  multiplication
  return x * y
*/
gf11_t gf11_mul(gf11_t x, gf11_t y);

/*
  reciplocal number
  return y where x * y = 1
 */
gf11_t gf11_recip(gf11_t x);

/*
  dividing
  return x / y
*/
gf11_t gf11_div(gf11_t x, gf11_t y);

/*
  negate
  return -x
*/
gf11_t gf11_neg(gf11_t x);

#define GF11_H 1
#endif
