/*
  Polynomial operation library
*/

#ifndef POLY_H

#include "gf.h"

#define POLY_SIZE GF_ORDER /* Maximum number of coefficients */

typedef struct POLY {
  int degree;
  gf_t coeff[POLY_SIZE];
} poly_t;

/*
  copy polynomial
*/
void poly_copy(poly_t *to, poly_t *from);

/*
  normalize polynomial degree
*/
void poly_normalize(poly_t *poly);

/*
  set polynomial degree by coefficients
*/
void poly_setdeg(poly_t *poly);


/*
  return true if polynomial is zero
*/
int poly_iszero(poly_t *poly);

/*
  assign zero to polynomial
*/
void poly_clear(poly_t *poly);

/*
  return true if polynomial is zero
*/
int poly_iszero(poly_t *poly);

/*
  polynomial division

  return
    quotient = dividend / divisor
    remainder = dividend % dvisor
*/
int poly_div(
	     poly_t *dividend,
	     poly_t *divisor,
	     poly_t *quotient,
	     poly_t *remainder
	     );


/*
  output polynomial coefficients
*/
void poly_print(poly_t *poly);



/* calculate polynomial value */
gf_t poly_subst(poly_t *poly, gf_t x);


/* polynomial addition */
void poly_add(poly_t *p1, poly_t *p2, poly_t *result);


/* polynomial subtraction */
void poly_sub(poly_t *p1, poly_t *p2, poly_t *result);


/* polynomial multiplication */
void poly_mul(poly_t *p1, poly_t *p2, poly_t *result);


/*
  extended Euclidean algorithm

  input x, y, t; t specifies degree of common divider
  output a, b, c where a*x + b*y = c (deg c < t)
*/
void poly_euclid(poly_t *x, poly_t *y, poly_t *a, poly_t *b, poly_t *c, int t);

/*
  negate polynomial
 */
void poly_neg(poly_t *poly);

/*
  Differentiating polynomial
 */
void poly_diff(poly_t *f, poly_t *df);

#define POLY_H 1
#endif
