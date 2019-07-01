#include <stdio.h>
#include <stdint.h>

#include "fx25tag.h"

uint64_t co_tag[CO_TAG_SIZE];

tag_t tags[CO_TAG_SIZE] = {
  { { .tag = 0 }, 0, 0 }, // Tag_00
  { { .tag = 0 }, 255, 239 },
  { { .tag = 0 }, 144, 128 },
  { { .tag = 0 }, 80, 64 },
  { { .tag = 0 }, 48, 32 }, // Tag_01 - Tag_04
  { { .tag = 0 }, 255, 223 },
  { { .tag = 0 }, 160, 128 },
  { { .tag = 0 }, 96, 64 },
  { { .tag = 0 }, 64, 32 }, // Tag_05 - Tag_08
  { { .tag = 0 }, 255, 191 },
  { { .tag = 0 }, 192, 128 },
  { { .tag = 0 }, 128, 64 }, // Tag_09 - Tag_0B
  { { .tag = 0 }, 255*2, 239*2 }, // Tag_0C - Tag_0F
  { { .tag = 0 }, 255*3, 239*3 },
  { { .tag = 0 }, 255*4, 239*4 },
  { { .tag = 0 }, 255*5, 239*5 },
};

static uint64_t gold_code(uint8_t iseed, uint8_t qseed)
{
  uint8_t ix = iseed;
  uint8_t qx = qseed;
  uint8_t ib, qb;
  uint8_t ic, qc;
  uint8_t qt;
  uint64_t ct = 0;
  int i;

  for (i = 0; i < CO_TAG_BITS; i++) {
    /* I(x) */
    ic = (ix & CARRY_BIT) != 0; // read x^6
    ib = ((ix ^ (ix << 1)) & CARRY_BIT) != 0; // I(x) = x^6 + x^5
    ix <<= 1;
    ix |= ib;

    /* Q(x) */
    qc = (qx & CARRY_BIT) != 0; // read x^6
    qt = (qx ^ (qx << 1));		       // Q(x) = x^6 + x^5 + x^3 + x^2
    qb = ((qt ^ (qt << 3)) & CARRY_BIT) != 0;
    qx <<= 1;
    qx |= qb;

    /* Correlation Tag */
    ct >>= 1;
    ct |= (uint64_t)(ic ^ qc) << 63;
  }
  return ct;
}

int fx25tag_init(void)
{
  int i;

  for (i = CO_TAG_00; i < CO_TAG_40; i++) {
    tags[i].tag = gold_code(I_SEED, i);
  }
  tags[CO_TAG_40].tag = gold_code(0x00, 0x3f); // Tag_40
  
  return 0;
}
