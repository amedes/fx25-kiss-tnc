/*
  Reed Solomon code library
*/

#ifndef RS_H

#include "gf.h"
#include "poly.h"

#define RS_ERR (-1)
#define RS_OK 0
#define GF_ELEMENTS 255

/*
  calculate generation polynamial

  n is maximum dimension of generation polynomial.
  return product((x - a^i), i, 0, n)
*/
int make_gp(poly_t *gp, int n);

/*
  initialize RS routine
*/
int rs_init(void);


/*
  Differentiating polynomial
 */
void poly_diff(poly_t *f, poly_t *df);


/*
  generate RS parity from message
*/
int rs_encode(gf_t code[], int code_len, int mesg_len);


/*
  decode RS code
*/
int rs_decode(gf_t code[], int code_len, int mesg_len);

#define RS_H 1
#endif
