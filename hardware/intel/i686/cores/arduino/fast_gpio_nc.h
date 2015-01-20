/*
 * fast_gpio_nc.c
 *
 * Implement a fast GPIO path.
 *
 * An explicit contract exists between this code and the in-kernel driver, since we both 'own'
 * the registers in question - user-space undertakes - never - ever to run concurrent data
 * whilst using the fast GPIO driver - in other words - user-space guarantees to never drive
 * traffic that can conflict with the kernel code.
 *
 * Author : David Hunt <dave@emutex.com> 2014
 */

#ifndef __FAST_GPIO_NC_H__
#define __FAST_GPIO_NC_H__

#include <Arduino.h>

#include "fast_gpio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Port offsets for Quark X1000 North-Cluster (Legacy) GPIO registers
#define QUARK_NC_CW_GPIO_REG			0x08
#define QUARK_NC_RW_GPIO_REG			0x28

// Wrapper macro to construct Quark X1000 NC core-well GPIO register descriptor
#define GPIO_FAST_ID_QUARK_NC_CW(mask)			    \
	GPIO_FAST_ID(GPIO_FAST_TYPE_QUARK_NC,		    \
		     QUARK_NC_CW_GPIO_REG,		    \
		     QUARK_NC_CW_GPIO_REG,		    \
		     (mask))

// Wrapper macro to construct Quark X1000 NC resume-well GPIO register descriptor
#define GPIO_FAST_ID_QUARK_NC_RW(mask)			    \
	GPIO_FAST_ID(GPIO_FAST_TYPE_QUARK_NC,		    \
		     QUARK_NC_RW_GPIO_REG,		    \
		     QUARK_NC_RW_GPIO_REG,		    \
		     (mask))

int fastGpioNCInit(void);
void fastGpioNCFini(void);
void fastGpioNCDigitalWrite(register uint8_t reg_offset, register uint8_t gpio_mask, register uint8_t val);
uint8_t fastGpioNCDigitalRead(uint8_t reg_offset, register uint8_t gpio_mask);
uint8_t fastGpioNCDigitalLatch(register uint8_t reg_offset);
void fastGpioNCDigitalWriteDestructive(register uint8_t reg_offset, register uint8_t gpio_levels);

#ifdef __cplusplus
}
#endif

#endif /* __FAST_GPIO_NC_H__ */
