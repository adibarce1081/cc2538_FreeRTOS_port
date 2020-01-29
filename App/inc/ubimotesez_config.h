
#ifndef __IOT_SECURITY_DEMO_H__
#define __IOT_SECURITY_DEMO_H__


typedef struct
{
    unsigned char mode;             //!< PER test mode. [RX|TX]
    unsigned char channel;          //!< PER test IEEE channel [11,26]
    unsigned char txPower;          //!< PER test TX power
    unsigned char gainMode;         //!< Gain mode
} moteCfg_t;

#define PAN_ID                  0xCDAC
#define TX_ADDR                 0x2538
#define RX_ADDR                 0xCCEB
#define CHANNEL			        0x1A

#define MOTE_MODE_TX             0
#define MOTE_MODE_RX             1

#define MOTE_GAIN_MODE_LO        0       // Same value as in hal_rf
#define MOTE_GAIN_MODE_HI        1       // Same value as in hal_rf
#define MOTE_GAIN_MODE_NONE      42
#define MAX_PAYLOAD		103


#endif
