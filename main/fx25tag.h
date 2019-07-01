#ifndef __FX25TAG_H__
#define __FX25TAG_H__

#include <stdint.h>

#define I_POLY 0x30
#define Q_POLY 0x36

#define CARRY_BIT 0x20

#define I_SEED 0x3f

#define CO_TAG_SIZE 0x41

#define CO_TAG_00 0x00
#define CO_TAG_01 0x01
#define CO_TAG_0B 0x0b
#define CO_TAG_0C 0x0c
#define CO_TAG_0E 0x0e
#define CO_TAG_0F 0x0f
#define CO_TAG_3F 0x3f
#define CO_TAG_40 0x40

#define CO_TAG_BITS 63

typedef struct {
  union {
    uint64_t tag;
    uint8_t byte[sizeof(uint64_t)];
  };
  int rs_code;
  int rs_info;
} tag_t;

extern tag_t tags[CO_TAG_SIZE];
extern uint64_t co_tag[CO_TAG_SIZE];

int fx25tag_init(void);

#endif /* __FX25TAG_H__ */
