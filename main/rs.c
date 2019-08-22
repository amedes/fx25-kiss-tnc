/*
  Reed Solomon code library
*/

#include <stdio.h>

#include "rs.h"
#include "poly.h"

#include "config.h"

static int rs_init_flag = 0;
//static int rs_code_len;
//static int rs_mesg_len;
//static int rs_gp_n;
//static int rs_t;
static poly_t rs_gen_p; // generating polynomial
static poly_t rs_gen_p16; // generating polynomial for 16 parity
static poly_t rs_gen_p32; // generating polynomial for 32 parity
static poly_t rs_gen_p64; // generating polynomial for 64 parity

#define RS_ERR (-1)
#define RS_OK 0
#define GF_ELEMENTS 255

/*
  calculate generating polynamial

  n is maximum dimension of generation polynomial.
  return product((x - a^i), i, 0, n)
*/
int make_gp(poly_t *gp, int n)
{
  int i, j;

  if (n >= POLY_SIZE) return -1; // too big

  poly_clear(gp);
  gp->coeff[0] = 1; // initialize

  for (i = 0; i < n; i++) {
    gf_t a = gf_pow(i);
    gf_t b = 0;

    for (j = 0; j <= i; j++) {
      gf_t c = gp->coeff[j]; /* save value */

      gp->coeff[j] = gf_sub(b, gf_mul(c, a)); /* a_i = a_i * a^i + a_i-1 */
      b = c; /* carry next degree of x */
    }
    gp->coeff[j] = b; /* increase most significant degree */
  }
  gp->degree = n;

  return 0;
}

/*
  make generating polynomial G_n(x)
  *
  * n: length of parity (16, 32, 64)
*/
static int make_genp(poly_t *gp, int n)
{
  int i;
  poly_t xa, tmp;
  gf_t an;

  if (n >= POLY_SIZE) return -1; /* too big */

  poly_clear(gp);
  gp->coeff[0] = 1; /* G_0(x) = 1 */

  /* initialize polynomial (x - a^n) */
  poly_clear(&xa);
  xa.degree = 1;
  xa.coeff[1] = 1;

  for (i = 0; i  < n; i++) {
    an = gf_pow(i); /* a^i */
    xa.coeff[0] = gf_neg(an); /* make (x - a^i) */

    poly_mul(gp, &xa, &tmp); /* G_i+i(x) = G_i(x) * (x - a^i) */
    poly_copy(gp, &tmp);
  }

  return 0;
}

/*
 * initialize Reed Solomon routine
 *
 * initialize Galoris feild routine
 * initialize generating polynomial
 */

int rs_init(void)
{
  if (rs_init_flag) return RS_OK;
  rs_init_flag = 1;

  /* initialize Galoris feild calculation routine */
  gf_init();

#if 0
  if (code_len <= 0) return RS_ERR;
  if (code_len > GF_ELEMENTS) return RS_ERR;

  rs_code_len = code_len;

  if (mesg_len >= code_len) return RS_ERR;
  if (mesg_len <= 0) return RS_ERR;

  rs_mesg_len = mesg_len;

  rs_gp_n = code_len - mesg_len; // length of parity
  rs_t = rs_gp_n / 2;

  if (rs_t <= 0) return RS_ERR;
#endif

  /* FX.25 uses only 16, 32 and 64 of parities */
  make_genp(&rs_gen_p16, 16);
  make_genp(&rs_gen_p32, 32);
  make_genp(&rs_gen_p64, 64);

  /*
  printf("rs_gen_p: %d\n", rs_gp_n);
  poly_print(&rs_gen_p);
  */

  return RS_OK;
}

/*
  generate RS parity from message
*/
int rs_encode(gf_t code[], int code_len, int mesg_len)
{
  int i;
  poly_t dividend, quotinent, remainder;
  int rs_code_len;
  int rs_mesg_len;
  int rs_gp_n;
  int rs_t;

  if (code_len <= 0) return RS_ERR;
  if (code_len > GF_ELEMENTS) return RS_ERR;

  rs_code_len = code_len;

  if (mesg_len >= code_len) return RS_ERR;
  if (mesg_len <= 0) return RS_ERR;

  rs_mesg_len = mesg_len;

  rs_gp_n = code_len - mesg_len; // length of parity
  rs_t = rs_gp_n / 2;

  if (rs_t <= 0) return RS_ERR;

  switch (rs_gp_n) {
    case 16:
      rs_gen_p = rs_gen_p16;
      break;
    case 32:
      rs_gen_p = rs_gen_p32;
      break;
    case 64:
      rs_gen_p = rs_gen_p64;
      break;
    default:
      return RS_ERR;
  }

  poly_clear(&dividend);

  /* copy message to dividend */
  for (i = 0; i < rs_mesg_len; i++) {
    dividend.coeff[(rs_code_len - 1) - i] = code[i];
  }
  dividend.degree = rs_code_len - 1;

  /* divide by generating polynomial */
  poly_div(&dividend, &rs_gen_p, &quotinent, &remainder);

  /* copy negated remainder */
  for (i = 0; i < rs_gp_n; i++) {
    code[rs_mesg_len + i] = gf_neg(remainder.coeff[(rs_gp_n - 1) - i]);
  }

  return RS_OK;
}

/*
  decode RS code
*/
int rs_decode(gf_t code[], int code_len, int mesg_len)
{
  int i;
  poly_t code_p, syn_p;
  poly_t quotient, remainder;
  int rs_code_len;
  int rs_mesg_len;
  int rs_gp_n;
  int rs_t;

  if (code_len <= 0) return RS_ERR;
  if (code_len > GF_ELEMENTS) return RS_ERR;

  rs_code_len = code_len;

  if (mesg_len >= code_len) return RS_ERR;
  if (mesg_len <= 0) return RS_ERR;

  rs_mesg_len = mesg_len;

  rs_gp_n = code_len - mesg_len; // length of parity
  rs_t = rs_gp_n / 2;

  if (rs_t <= 0) return RS_ERR;

  switch (rs_gp_n) {
    case 16:
      rs_gen_p = rs_gen_p16;
      break;
    case 32:
      rs_gen_p = rs_gen_p32;
      break;
    case 64:
      rs_gen_p = rs_gen_p64;
      break;
    default:
      return RS_ERR;
  }

  /* make polynomial from received code */
  for (i = 0; i < rs_code_len; i++) {
    code_p.coeff[(rs_code_len - 1) - i] = code[i];
  }
  code_p.degree = rs_code_len - 1;

 
  /* calculate syndrome */

  /*
  printf("code_p\n");
  poly_print(&code_p);

  printf("Syndrome\n");
  */

  poly_clear(&syn_p);
  for (i = 0; i < rs_gp_n; i++) { // rs_gp_n is number of parity
    gf_t s = poly_subst(&code_p, gf_pow(i));
    syn_p.coeff[(rs_gp_n - 1) - i] = s;
    /*printf("C(a^%d) = %d, a^%d\n", i, s, gf_ind(s));*/
  }
  syn_p.degree = rs_gp_n - 1;
  poly_normalize(&syn_p);

  if (poly_iszero(&syn_p)) {
    /* parity check */
    poly_div(&code_p, &rs_gen_p, &quotient, &remainder);
    if (poly_iszero(&remainder)) return 0; // no error

      return -1; /* something wrong */
  }

  /*
   * do error correction
   */

  poly_t x_2t;

  poly_clear(&x_2t);
  x_2t.degree = rs_gp_n;
  x_2t.coeff[rs_gp_n] = 1;

  poly_t sigma, omega;
  poly_t phi;

  /*
  printf("x^(2*t)\n");
  poly_print(&x_2t);
  printf("S(x)\n");
  poly_print(&syn_p);
  */

  /* -S(x) */
  /*
  poly_neg(&syn_p);
  printf("-S(x)\n");
  poly_print(&syn_p);
  */

  /* calculate sigma(x), omega(x) */
  poly_euclid(&x_2t, &syn_p, &sigma, &omega, &phi, rs_gp_n / 2);

  /*
  printf("sigma(x)\n");
  poly_print(&sigma);
  printf("omega(x)\n");
  poly_print(&omega);

  printf("phi(x)\n");
  poly_print(&phi);

  poly_t t0, t1, t2;

  poly_mul(&x_2t, &omega, &t0);
  poly_mul(&syn_p, &sigma, &t1);
  poly_add(&t0, &t1, &t2);

  printf("a*x + b*y =\n");
  poly_print(&t2);
  */

  /* error correction */

  /* need -sigma(x) */
  poly_neg(&sigma);

  poly_t dsigma;

  /* differentiating sigma */
  poly_diff(&sigma, &dsigma);

  /*
  printf("dsigma(x)\n");
  poly_print(&dsigma);
  */

  /* find error position */
  int errs = 0;
  /*printf("find error position\n");*/
  for (i = 0; (i < rs_code_len) && (errs < rs_t); i++) {
    gf_t an = gf_pow(i);

    if (poly_subst(&sigma, an) == 0) { // sigma(an) == 0, "i" is error position
      gf_t c, e, n;

      /*printf("sigma(a^%d) = 0\n", i);*/

      /* calculate error value */
      e = gf_div(poly_subst(&omega, an), poly_subst(&dsigma, an));

      //if (e == 0) continue; // no need to correct

      errs++;
      /* correct message */
      c = code[(rs_code_len - 1) - i];
      n = gf_sub(c, e);
      code[(rs_code_len - 1) - i] = n;
      code_p.coeff[i] = n; // for parity check

#if 0
      printf("i: %d, an: %d, omega(%d) = %d, dsigma(%d) = %d, e = %02x, c = %02x, n = %02x\n", i, an, 
	     an, poly_subst(&omega, an), an, poly_subst(&dsigma, an), e, c, n);
#endif

      if (c == n) printf("rs: error correction return same value: e = %02x, (%02x->%02x)\n", e, c, n);

      /*printf("m[%d]: old: %d, err: %d, new: %d\n", i, c, e, n);*/

    }
  }

  if (errs == 0) { /* something wrong */
    return -1;
  }

  /* parity check */
  poly_div(&code_p, &rs_gen_p, &quotient, &remainder);
  if (!poly_iszero(&remainder)) return -2; // parity error not corrected 

  return errs;
}
