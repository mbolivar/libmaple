/* insert license */

#include "gpio.h"

#ifndef __USB_CONFIG_H
#define __USB_CONFIG_H

#define RESET_DELAY               (100000)
#define USB_CONFIG_MAX_POWER      (100 >> 1)

/* choose addresses to give endpoints the max 64 byte buffers */
#define USB_BTABLE_ADDRESS        0x00

#define bMaxPacketSize            0x40  /* 64B, maximum for USB FS Devices */

#define NUM_ENDPTS                0x04

/* handle CTRM, WKUPM, SUSPM, ERRM, SOFM, ESOFM, RESETM */
#define ISR_MSK                   0xBF00

#define F_SUSPEND_ENABLED 1

#endif
