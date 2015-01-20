/*
 * fast_gpio_sc.c
 *
 * Implement a fast GPIO path.
 *
 * An explicit contract exists between this code and the in-kernel driver, since we both 'own'
 * the registers in question - user-space undertakes - never - ever to run concurrent data
 * whilst using the fast GPIO driver - in other words - user-space guarantees to never drive
 * traffic that can conflict with the kernel code.
 *
 * This a write only interface - read is verboten - verboten !
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@intel.com> 2013
 */

#ifndef __FAST_GPIO_SC_H__
#define __FAST_GPIO_SC_H__

#include <Arduino.h>

#include "fast_gpio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register offsets for Quark X1000 South-Cluster (GIP) GPIO registers
#define QUARK_SC_GPIO_REG_OUT			0x00
#define QUARK_SC_GPIO_REG_IN			0x50

// Wrapper macro to construct Quark X1000 SC GPIO register descriptor
#define GPIO_FAST_ID_QUARK_SC(mask)			    \
	GPIO_FAST_ID(GPIO_FAST_TYPE_QUARK_SC,		    \
		     QUARK_SC_GPIO_REG_IN,		    \
		     QUARK_SC_GPIO_REG_OUT,		    \
		     (mask))

int fastGpioSCInit(void);
void fastGpioSCFini(void);
void fastGpioSCDigitalWrite(register uint8_t reg_offset, register uint8_t gpio, register uint8_t val);
uint8_t fastGpioSCDigitalRead(register uint8_t reg_offset, register uint8_t gpio);
void fastGpioSCDigitalWriteDestructive(register uint8_t reg_offset, register uint8_t gpio);
uint32_t fastGpioSCDigitalLatch(register uint8_t reg_offset);

#ifdef __cplusplus
}
#endif

#endif /* __FAST_GPIO_SC_H__ */
