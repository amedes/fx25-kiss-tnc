/*
 * tx_cntl.h
 *
 * control radio transmitter
 */
#include <stdint.h>

#define PTT_ON 1
#define PTT_OFF 0

#define AX25_MODE 0
#define FX25_PARITY_16 16
#define FX25_PARITY_32 32
#define FX25_PARITY_64 64

#define FX25_SETMODE 0

void wait_carrier_inactive(void);
void wait_txdelay(void);
int p_persistent(void);
void wait_slottime(void);
void set_fullduplex(int ch, int duplex);
void set_txdelay(int ch, int delay);
void set_parameter_p(int ch, int p);
void set_slottime(int ch, int time);
void set_sethardware(int ch, uint8_t buf[], int size);
int get_tnc_mode(void);
