#include <stdio.h>
#include <stdint.h>

#include "fx25tag.h"

uint64_t co_tag[CO_TAG_SIZE];

tag_t tags[CO_TAG_SIZE] = {
    { { .tag = 0 }, 0, 0, 0, 0 }, // Tag_00
    { { .tag = 0 }, 1, 1, 255, 239 }, 
    { { .tag = 0 }, 1, 1, 144, 128 },
    { { .tag = 0 }, 1, 1, 80, 64 },
    { { .tag = 0 }, 1, 1, 48, 32 }, // Tag_01 - Tag_04
    { { .tag = 0 }, 1, 1, 255, 223 },
    { { .tag = 0 }, 1, 1, 160, 128 },
    { { .tag = 0 }, 1, 1, 96, 64 },
    { { .tag = 0 }, 1, 1, 64, 32 }, // Tag_05 - Tag_08
    { { .tag = 0 }, 1, 1, 255, 191 },
    { { .tag = 0 }, 1, 1, 192, 128 },
    { { .tag = 0 }, 1, 1, 128, 64 }, // Tag_09 - Tag_0B

    { { .tag = 0 }, 1, 2, 255, 239 }, // Tag_0C - Tag_0F
    { { .tag = 0 }, 1, 2, 255, 223 },
    { { .tag = 0 }, 1, 2, 255, 191 },
    { { .tag = 0 }, 1, 3, 255, 191 }, 

    { { .tag = 0 }, 0, 0, 256, 256 }, // Tag_10
    { { .tag = 0 }, 2, 1, 256, 256 }, 
    { { .tag = 0 }, 2, 2, 256, 256 }, 
    { { .tag = 0 }, 2, 3, 256, 256 }, 
    { { .tag = 0 }, 2, 4, 256, 256 }, 
    { { .tag = 0 }, 2, 5, 256, 256 }, 
    { { .tag = 0 }, 2, 6, 256, 256 }, 
    { { .tag = 0 }, 2, 7, 256, 256 }, 
    { { .tag = 0 }, 2, 8, 256, 256 }, 
    { { .tag = 0 }, 2, 9, 256, 256 }, 
    { { .tag = 0 }, 2,10, 256, 256 }, 
    { { .tag = 0 }, 2,11, 256, 256 }, 
    { { .tag = 0 }, 2,12, 256, 256 }, 
    { { .tag = 0 }, 2,13, 256, 256 }, 
    { { .tag = 0 }, 2,14, 256, 256 }, 
    { { .tag = 0 }, 2,15, 256, 256 }, // Tag_1F
  
    { { .tag = 0 }, 3, 0, 0, 0 },     // Tag_20 CODE TAG BASE
    { { .tag = 0 }, -1,0, 0, 0 },     // sentinel
};

tag_t code_tags[CODE_TAG_SIZE] = {
    { { .tag = 0 }, 0, 0, 0, 0 },     // CODE_Tag_00 - Tag_03
    { { .tag = 0 }, 3, 0, 255, 239 }, 
    { { .tag = 0 }, 3, 0, 255, 223 },
    { { .tag = 0 }, 3, 0, 255, 191 },
    { { .tag = 0 }, -1,0, 0, 0 },     // sentinel
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
    static int initialized = 0;

    if (initialized == 0) {
        int i;
        initialized = 1;

        for (i = CO_TAG_00; i < CO_TAG_40; i++) {
            tags[i].tag = gold_code(I_SEED, i);
        }
        tags[CO_TAG_40].tag = gold_code(0x00, 0x3f); // Tag_40

        uint64_t code_tag = tags[CODE_TAG_BASE].tag;
        for (i = 0; i < CODE_TAG_SIZE; i++) {
            code_tags[i].tag = code_tag;
            code_tag >>= 1;
            code_tag |= (code_tag & 1) << 63;
            code_tag &= 0xFFFFFFFFFFFFFFFE;
        }

    }
    return 0;
}
