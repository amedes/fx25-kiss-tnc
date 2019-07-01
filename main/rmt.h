#include <stdint.h>

#if 1
typedef union {
    struct {
	uint16_t duration :15;
	uint16_t level :1;
    };
    uint16_t val;
} rmt_item16_t;
#endif

void rmt_tx_init(void);
void send_packet(uint8_t data[], int len, int wait);
