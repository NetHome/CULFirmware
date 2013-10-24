#ifndef __FHT_H
#define __FHT_H

void fhtsend(char *in);
void fht_init(void);
void fht_hook(uint8_t *in);
void fht8v_timer(void);
void fht80b_timer(void);

extern uint8_t fht8v_timeout;
extern uint8_t fht80b_timeout;
extern uint8_t fht_hc[2];

#define FHT_ACTUATOR   0x00
#define FHT_ACK        0x4B
#define FHT_CAN_XMIT   0x53
#define FHT_CAN_RCV    0x54
#define FHT_ACK2       0x69
#define FHT_START_XMIT 0x7D
#define FHT_END_XMIT   0x7E
#define FHT_MINUTE     0x64

#define FHT_DATA       0xff
#define FHT_FOREIGN    0xff

#define FHT_CSUM_START   12
#define FHT_NMSG          2



#endif
