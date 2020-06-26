/*
  polynomial operation library
*/

#include <stdio.h>

#include "gf.h"
#include "poly.h"

//#define POLY_STATIC 1 // allocate work area as static variable

/*
  copy polynomial
*/
void poly_copy(poly_t *to, poly_t *from)
{
  int i;

  for (i = 0; i <= from->degree; i++)
    to->coeff[i] = from->coeff[i];

  to->degree = from->degree;
}

/*
  normalize polynomial degree
*/
void poly_normalize(poly_t *poly)
{
  int i;

  i = poly->degree;
  while ((i > 0) && (poly->coeff[i] == 0)) {
    --i;
  }
  poly->degree = i;
}

/*
  set polynomial degree by coefficients
*/
void poly_setdeg(poly_t *poly)
{
  int i;

  //i = poly->degree;
  i = POLY_SIZE - 1;
  while ((i > 0) && (poly->coeff[i] == 0)) {
    --i;
  }
  poly->degree = i;
}

/*
  assign zero to polynomial
*/
void poly_clear(poly_t *poly)
{
  int i;

  for (i = 0; i < POLY_SIZE; i++)
    poly->coeff[i] = 0;

  poly->degree = 0;
}

/*
  return true if polynomial is zero
*/
int poly_iszero(poly_t *poly)
{
  int i;

  i = poly->degree;
  while ((i > 0) && (poly->coeff[i] == 0)) {
    --i;
  }

  return (i == 0) && (poly->coeff[i] == 0);
}

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
	     )
{
  int i, j;
  poly_t dd; /* work for divide */

  if (poly_iszero(divisor)) return -1; // dvisor == 0

  /* degree of quotient */
  int qi = dividend->degree - divisor->degree;

  /* if quotient == 0 */
  if (qi < 0) {
      quotient->degree = 0;
      quotient->coeff[0] = 0;
      poly_copy(remainder, dividend);

      return 0;
  }

  quotient->degree = qi;
  poly_copy(&dd, dividend);

  /* reciplocal number of most significant coefficient of divisor */
  gf_t d = divisor->coeff[divisor->degree];

  for (i = dd.degree; i >= divisor->degree; --i) {
    gf_t q = gf_div(dd.coeff[i], d); /* quotient = dd.coeff / divisor.coeff */
    quotient->coeff[qi--] = q;

    /* subtract by divisor * q */
    for (j = 0; j <= divisor->degree; j++) {
      dd.coeff[i - divisor->degree + j] =
	gf_sub(dd.coeff[i - divisor->degree + j], gf_mul(divisor->coeff[j], q));
    }
  }

  poly_copy(remainder, &dd); // dd is remainder
  poly_normalize(remainder); // normalize remainder and quotinent
  poly_normalize(quotient);

  return 0;
}

/*
  output polynomial coefficients
*/
void poly_print(poly_t *poly)
{
  int i;
  gf_t c;

  printf("degree: %d\n", poly->degree);
  for (i = 0; i <= poly->degree; i++) {
    c = poly->coeff[i];
    if ((c != 0) || (i == 0)) printf("coeff[%d]: %d, a^%d\n", i, c, gf_ind(c));
  }
}


/* calculate polynomial value */
gf_t poly_subst(poly_t *poly, gf_t x)
{
  int i;
  gf_t v = 0;

  for (i = poly->degree; i >= 0; --i) {
    v = gf_add(gf_mul(v, x), poly->coeff[i]);
  }

  return v;
}

/*
  polynomial addition/subtraction
*/
static void poly_addsub(poly_t *p1, poly_t *p2, poly_t *result,
		     gf_t (*gf_addsub)(gf_t x, gf_t y))
{
  int i;

  poly_clear(result);

  i = 0;
  while ((i <= p1->degree) || (i <= p2->degree)) {
    result->coeff[i] = (*gf_addsub)((i <= p1->degree) ? p1->coeff[i] : 0,
			       (i <= p2->degree) ? p2->coeff[i] : 0);
    //printf("poly_add: coeff[%d] = %d\n", i, result->coeff[i]);
    i++;
  }

  result->degree = i - 1;
  poly_normalize(result);
  //printf("poly_add: degree: %d\n", result->degree);
}

/* polynomial addtion */
void poly_add(poly_t *p1, poly_t *p2, poly_t *result)
{
  poly_addsub(p1, p2, result, gf_add);
}

/* polynomial subtraction */
void poly_sub(poly_t *p1, poly_t *p2, poly_t *result)
{
  poly_addsub(p1, p2, result, gf_sub);
}

/* polynomial multiplication */
void poly_mul(poly_t *p1, poly_t *p2, poly_t *result)
{
  int i, j;

  poly_clear(result);

  for (i = 0; i <= p1->degree; i++) {
    gf_t m = p1->coeff[i];
    for (j = 0; j <= p2->degree; j++) {
      gf_t t;
      t = gf_mul(m, p2->coeff[j]);
      result->coeff[i + j] = gf_add(result->coeff[i + j], t);
    }
  }
  result->degree = p1->degree + p2->degree;
  poly_normalize(result);
}

/*
  extended Euclidean algorithm

  input x, y, t; t specifies degree of remainder
  output a, b, c where a*x + b*y = c (deg c < t)
*/
void poly_euclid(poly_t *x, poly_t *y, poly_t *a, poly_t *b, poly_t *c, int t)
{
#ifdef POLY_STATIC
  static poly_t a0s, a1s, a2s;
  static poly_t b0s, b1s, b2s;
  static poly_t r0s, r1s, r2s;
  static poly_t q1s;
  static poly_t tmps;
#endif

  poly_t *a0, *a1, *a2;
  poly_t *b0, *b1, *b2;
  poly_t *r0, *r1, *r2;
  poly_t *q1;
  poly_t *tmp;

#ifdef POLY_STATIC
  a0 = &a0s;
  a1 = &a1s;
  a2 = &a2s;
  b0 = &b0s;
  b1 = &b1s;
  b2 = &b2s;
  r0 = &r0s;
  r1 = &r1s;
  r2 = &r2s;
  q1 = &q1s;
  tmp = &tmps;
#else
  // allocate work memory by malloc
  a0 = malloc(sizeof(poly_t));
  a1 = malloc(sizeof(poly_t));
  a2 = malloc(sizeof(poly_t));
  b0 = malloc(sizeof(poly_t));
  b1 = malloc(sizeof(poly_t));
  b2 = malloc(sizeof(poly_t));
  r0 = malloc(sizeof(poly_t));
  r1 = malloc(sizeof(poly_t));
  r2 = malloc(sizeof(poly_t));
  q1 = malloc(sizeof(poly_t));
  tmp = malloc(sizeof(poly_t));

  if (!(a0 && a1 && a2 && b0 && b1 && b2 && r0 && r1 && r2 && q1 && tmp)) {
    fprintf(stderr, "poly_euclid(): cannot allocate memory\n");
    abort();
  }
#endif

  //printf("t = %d\n", t);

  poly_copy(r0, x);
  poly_copy(r1, y);

  poly_clear(a0); a0->coeff[0] = 1;
  poly_clear(a1);
  poly_clear(b0);
  poly_clear(b1); b1->coeff[0] = 1;

  while (r1->degree >= t) { /* exit if deg r1 < t */

    poly_div(r0, r1, q1, r2); /* q1 = r0 / r1 and r2 = r0 % r1 */


    /*
    printf("quotient\n");
    poly_print(&q1);

    printf("remainder\n");
    poly_print(&r2);
    */

    poly_mul(q1, a1, tmp); // tmp = aq * a1
    poly_sub(a0, tmp, a2); // a2 = a0 - tmp

    poly_mul(q1, b1, tmp); // tmp = q1 * b1
    poly_sub(b0, tmp, b2); // b2 = b0 - tmp

    poly_copy(r0, r1); // r0 = r1
    poly_copy(r1, r2); // r1 = r2

    poly_copy(a0, a1); // a0 = a1
    poly_copy(a1, a2); // a1 = a2

    poly_copy(b0, b1); // b0 = b1
    poly_copy(b1, b2); // b1 = b2
  }

  poly_copy(c, r1); // c = r0
  poly_copy(a, b1); // a = a0
  poly_copy(b, a1); // b = b0

#ifndef POLY_STATIC
  free(a0);
  free(a1);
  free(a2);
  free(b0);
  free(b1);
  free(b2);
  free(r0);
  free(r1);
  free(r2);
  free(q1);
  free(tmp);
#endif
}

/*
  negate polynomial
 */
void poly_neg(poly_t *poly)
{
  int i;

  for (i = 0; i <= poly->degree; i++) {
    poly->coeff[i] = gf_neg(poly->coeff[i]); /* coeff = - coeff */
  }
}

/*
  Differentiating polynomial
 */
void poly_diff(poly_t *f, poly_t *df)
{
  int i;

  poly_clear(df);

  if (f->degree <= 0) return;

  for (i = 1; i <= f->degree; i++) {
    gf_t c = 0;
    gf_t a = f->coeff[i];
    int j = i;

    while (j-- > 0) { /* add i times (can not use gf_mul) */
      c = gf_add(c, a);
    }

    df->coeff[i - 1] = c; /* coefficient of x^(i-1) */
  }
  df->degree = f->degree - 1;
  poly_normalize(df);
}
