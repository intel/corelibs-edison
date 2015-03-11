/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2014 Intel Corporation.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <Arduino.h>
#include <interrupt.h>
#include <Mux.h>
#include <sysfs.h>
#include <trace.h>
#include "variant.h"

//Bindings to Arduino
#include "RingBuffer.h"
#include "TTYUART.h"
#include "fast_gpio_pci.h"

#define MY_TRACE_PREFIX "variant"

#ifdef __cplusplus
extern "C" {
#endif

#define GP12_PWM0			12
#define GP13_PWM1			13
#define GP182_PWM2			182
#define GP183_PWM3			183
#define any_equal(a,b,c,d)		(a==b||a==c||a==d||b==c||b==d||c==d)
#define all_pwms(a,b,c,d)		(digitalPinHasPWM(a)&&digitalPinHasPWM(b)&&\
					digitalPinHasPWM(c)&&digitalPinHasPWM(d))

struct pwm_muxing {
	uint8_t gpioid;
	mux_sel_t *muxing;
};

const int mux_sel_analog[NUM_ANALOG_INPUTS] = {
	MUX_SEL_AD7298_VIN0,
	MUX_SEL_AD7298_VIN1,
	MUX_SEL_AD7298_VIN2,
	MUX_SEL_AD7298_VIN3,
	MUX_SEL_AD7298_VIN4,
	MUX_SEL_AD7298_VIN5,
};

const int mux_sel_uart[NUM_UARTS][MUX_DEPTH_UART] = {
	/* This is auto-indexed (board pinout) */
	{MUX_SEL_NONE, MUX_SEL_NONE},				// ttyGS0 - USB not muxed
	{MUX_SEL_UART0_RXD,	MUX_SEL_UART0_TXD},		// ttyS0 - muxed
};

const int  mux_sel_spi[NUM_SPI][MUX_DEPTH_SPI] = {
	{
		MUX_SEL_NONE,
		MUX_SEL_NONE,
		MUX_SEL_NONE
	},
	{
		MUX_SEL_SPI1_MOSI,
		MUX_SEL_SPI1_MISO,
		MUX_SEL_SPI1_SCK
	},
};

const int  mux_sel_i2c[NUM_I2C][MUX_DEPTH_I2C] = {
	{
		MUX_SEL_I2C_SCL,
		MUX_SEL_I2C_SDA,
	},
};

mux_sel_t MuxDesc0[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_UART }, // Tristate disabled for all pins
	{ 248, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 248, LOW, FN_GPIO_INPUT | FN_UART}, // Output disabled
	{ 216, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 216, LOW, FN_GPIO_INPUT_PULLDOWN | FN_UART }, // Pulldown enabled
	{ 216, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ }, // Pullup disabled
	{ 130, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 130, PIN_MODE_1, FN_UART }, // UART mode
	{ 214, LOW, FN_GPIO | FN_UART } // Tristate enabled for all pins
};

mux_sel_t MuxDesc1[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_UART }, // Tristate disabled for all pins
	{ 249, HIGH, FN_GPIO_OUTPUT | FN_UART }, // Output enabled
	{ 249, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 217, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 217, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 217, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_UART }, // Pullup disabled
	{ 131, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 131, PIN_MODE_1, FN_UART }, // UART mode
	{ 214, LOW, FN_GPIO | FN_UART } // Tristate enabled for all pins
};

mux_sel_t MuxDesc2[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO }, // Tristate disabled for all pins
	{ 250, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 250, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 218, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enabled
	{ 218, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 218, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ }, // Pullup disabled
	{ 128, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO } // Tristate enabled for all pins
};

mux_sel_t MuxDesc3[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_PWM }, // Tristate disabled for all pins
	{ 251, HIGH, FN_GPIO_OUTPUT | FN_PWM }, // Output enabled
	{ 251, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 219, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enabled
	{ 219, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 219, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_PWM }, // Pullup disabled
	{  12, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO | FN_PWM } // Tristate enabled for all pins
};

mux_sel_t MuxDesc4[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO }, // Tristate disabled for all pins
	{ 252, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 252, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 220, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enabled
	{ 220, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 220, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ }, // Pullup disabled
	{ 129, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO } // Tristate enabled for all pins
};

mux_sel_t MuxDesc5[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_PWM }, // Tristate disabled for all pins
	{ 253, HIGH, FN_GPIO_OUTPUT | FN_PWM }, // Output enabled
	{ 253, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 221, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 221, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 221, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_PWM }, // Pullup disabled
	{  13, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO | FN_PWM } // Tristate enabled for all pins
};

mux_sel_t MuxDesc6[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_PWM }, // Tristate disabled for all pins
	{ 254, HIGH, FN_GPIO_OUTPUT | FN_PWM }, // Output enabled
	{ 254, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 222, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 222, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 222, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_PWM }, // Pullup disabled
	{ 182, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO | FN_PWM } // Tristate enabled for all pins
};

mux_sel_t MuxDesc7[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO }, // Tristate disabled for all pins
	{ 255, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 255, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 223, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 223, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 223, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ }, // Pullup disabled
	{  48, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO } // Tristate enabled for all pins
};

mux_sel_t MuxDesc8[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO }, // Tristate disabled for all pins
	{ 256, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 256, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 224, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 224, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 224, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ }, // Pullup disabled
	{  49, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO } // Tristate enabled for all pins
};

mux_sel_t MuxDesc9[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_PWM }, // Tristate disabled for all pins
	{ 257, HIGH, FN_GPIO_OUTPUT | FN_PWM }, // Output enabled
	{ 257, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 225, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 225, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 225, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_PWM }, // Pullup disabled
	{ 183, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO | FN_PWM } // Tristate enabled for all pins
};

mux_sel_t MuxDesc10[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_PWM }, // Tristate disabled for all pins
	{ 258, HIGH, FN_GPIO_OUTPUT | FN_PWM }, // Output enabled
	{ 258, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 226, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 226, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 226, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_PWM }, // Pullup disabled
	{ 240, LOW, FN_GPIO }, // Mux Control
	{ 263, HIGH, FN_GPIO }, // Mux Control
	{ 263, LOW, FN_PWM }, // Mux Control
	{  41, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 214, LOW, FN_GPIO | FN_PWM} // Tristate enabled for all pins
};

mux_sel_t MuxDesc11[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_SPI | FN_PWM }, // Tristate disabled for all pins
	{ 259, HIGH, FN_GPIO_OUTPUT | FN_SPI | FN_PWM }, // Output enabled
	{ 259, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 227, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 227, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 227, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_SPI }, // Pullup disabled
	{ 241, LOW, FN_GPIO }, // Mux Control
	{ 241, HIGH, FN_SPI }, // Mux Control
	{ 262, LOW, FN_PWM }, // Mux Control
	{ 262, HIGH, FN_GPIO | FN_SPI }, // Mux Control
	{  43, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 115, PIN_MODE_1, FN_SPI }, // SPI mode
	{ 214, LOW, FN_GPIO | FN_SPI | FN_PWM } // Tristate enabled for all pins
};

mux_sel_t MuxDesc12[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_SPI }, // Tristate disabled for all pins
	{ 260, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 260, LOW, FN_GPIO_INPUT | FN_SPI }, // Output disabled
	{ 228, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 228, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 228, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_SPI }, // Pullup disabled
	{ 242, LOW, FN_GPIO }, // Mux Control
	{ 242, HIGH, FN_SPI }, // Mux Control
	{  42, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 114, PIN_MODE_1, FN_SPI }, // SPI mode
	{ 214, LOW, FN_GPIO | FN_SPI } // Tristate enabled for all pins
};

mux_sel_t MuxDesc13[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_SPI }, // Tristate disabled for all pins
	{ 261, HIGH, FN_GPIO_OUTPUT | FN_SPI }, // Output enabled
	{ 261, LOW, FN_GPIO_INPUT }, // Output disabled
	{ 229, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 229, LOW, FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 229, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_SPI }, // Pullup disabled
	{ 243, LOW, FN_GPIO }, // PinMux mode
	{ 243, HIGH, FN_SPI }, // PinMux mode
	{  40, PIN_MODE_0, FN_GPIO }, // GPIO mode
	{ 109, PIN_MODE_1, FN_GPIO }, // SPI mode
	{ 214, LOW, FN_GPIO | FN_SPI } // Tristate enabled for all pins
};

mux_sel_t MuxDesc14[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_ANALOG }, // Tristate disabled for all pins
	{ 200, LOW,  FN_GPIO }, // Mux control
	{ 200, HIGH, FN_ANALOG }, // Mux control
	{ 232, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 232, LOW,  FN_GPIO_INPUT | FN_ANALOG }, // Output disabled
	{ 208, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 208, LOW,  FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 208, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_ANALOG }, // Pullup disabled
	{  44, PIN_MODE_0, FN_GPIO | FN_ANALOG }, 
	{ 214, LOW,  FN_GPIO | FN_ANALOG } // Tristate enabled for all pins
};

mux_sel_t MuxDesc15[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_ANALOG }, // Tristate disabled for all pins
	{ 201, LOW,  FN_GPIO }, // Mux control
	{ 201, HIGH, FN_ANALOG }, // Mux control
	{ 233, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 233, LOW,  FN_GPIO_INPUT | FN_ANALOG }, // Output disabled
	{ 209, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 209, LOW,  FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 209, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_ANALOG }, // Pullup disabled
	{  45, PIN_MODE_0, FN_GPIO | FN_ANALOG }, 
	{ 214, LOW,  FN_GPIO | FN_ANALOG } // Tristate enabled for all pins
};

mux_sel_t MuxDesc16[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_ANALOG }, // Tristate disabled for all pins
	{ 202, LOW,  FN_GPIO }, // Mux control
	{ 202, HIGH, FN_ANALOG }, // Mux control
	{ 234, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 234, LOW,  FN_GPIO_INPUT | FN_ANALOG }, // Output disabled
	{ 210, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 210, LOW,  FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 210, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_ANALOG }, // Pullup disabled
	{  46, PIN_MODE_0, FN_GPIO | FN_ANALOG }, 
	{ 214, LOW,  FN_GPIO | FN_ANALOG } // Tristate enabled for all pins
};

mux_sel_t MuxDesc17[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_ANALOG }, // Tristate disabled for all pins
	{ 203, LOW,  FN_GPIO }, // Mux control
	{ 203, HIGH, FN_ANALOG }, // Mux control
	{ 235, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 235, LOW,  FN_GPIO_INPUT | FN_ANALOG }, // Output disabled
	{ 211, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 211, LOW,  FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 211, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_ANALOG }, // Pullup disabled
	{  47, PIN_MODE_0, FN_GPIO | FN_ANALOG }, 
	{ 214, LOW,  FN_GPIO | FN_ANALOG } // Tristate enabled for all pins
};

mux_sel_t MuxDesc18[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_I2C | FN_ANALOG }, // Tristate disabled for all pins
	{  14, NONE, FN_I2C }, // Disable GPIO output for I2C
	{ 204, LOW,  FN_GPIO | FN_I2C }, // Mux control
	{ 204, HIGH, FN_ANALOG }, // Mux control
	{ 236, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 236, LOW,  FN_GPIO_INPUT | FN_I2C | FN_ANALOG }, // Output disabled
	{ 212, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 212, LOW,  FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 212, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_I2C | FN_ANALOG }, // Pullup disabled
	{  14, PIN_MODE_0, FN_GPIO | FN_ANALOG }, 
	{  27, PIN_MODE_1, FN_I2C }, // I2C mode
	{ 214, LOW,  FN_GPIO | FN_I2C | FN_ANALOG } // Tristate enabled for all pins
};

mux_sel_t MuxDesc19[] = {
	//gpio, value, type
	{ 214, HIGH, FN_GPIO | FN_I2C | FN_ANALOG }, // Tristate disabled for all pins
	{ 165, NONE, FN_I2C }, // Disable GPIO output for I2C
	{ 205, LOW,  FN_GPIO | FN_I2C }, // Mux control
	{ 205, HIGH, FN_ANALOG }, // Mux control
	{ 237, HIGH, FN_GPIO_OUTPUT }, // Output enabled
	{ 237, LOW,  FN_GPIO_INPUT | FN_I2C | FN_ANALOG }, // Output disabled
	{ 213, HIGH, FN_GPIO_INPUT_PULLUP }, // Pullup enabled
	{ 213, LOW,  FN_GPIO_INPUT_PULLDOWN }, // Pulldown enabled
	{ 213, NONE, FN_GPIO_OUTPUT | FN_GPIO_INPUT_HIZ | FN_I2C | FN_ANALOG }, // Pullup disabled
	{ 165, PIN_MODE_0, FN_GPIO | FN_ANALOG }, 
	{ 28,  PIN_MODE_1, FN_I2C }, // I2C mode
	{ 214, LOW,  FN_GPIO | FN_I2C | FN_ANALOG } // Tristate enabled for all pins
};



// Sorted by Linux GPIO ID
PinDescription g_APinDescription[] =
{
//	gpiolib	alias	fastinf	ardid	Initial			FixdSt	ptMuxDesc,		MuxCount		type		Handle	extPU	iAlt	pAlt
	{ 12,	NONE,	3,	3,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc3,	MUX_SIZE(MuxDesc3),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO3
	{ 13,	NONE,	5,	5,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc5,	MUX_SIZE(MuxDesc5),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO5
	{ 14,	NONE,	18,	18,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc18,	MUX_SIZE(MuxDesc18),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO18 (AIN4)
	{ 40,	NONE,	13,	13,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc13,	MUX_SIZE(MuxDesc13),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO13
	{ 41,	NONE,	10,	10,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc10,	MUX_SIZE(MuxDesc10),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO10
	{ 42,	NONE,	12,	12,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc12,	MUX_SIZE(MuxDesc12),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO12
	{ 43,	NONE,	11,	11,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc11,	MUX_SIZE(MuxDesc11),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO11
	{ 44,	NONE,	14,	14,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc14,	MUX_SIZE(MuxDesc14),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO14 (AIN0)
	{ 45,	NONE,	15,	15,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc15,	MUX_SIZE(MuxDesc15),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO15 (AIN1)
	{ 46,	NONE,	16,	16,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc16,	MUX_SIZE(MuxDesc16),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO16 (AIN2)
	{ 47,	NONE,	17,	17,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc17,	MUX_SIZE(MuxDesc17),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO17 (AIN3)
	{ 48,	NONE,	7,	7,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc7,	MUX_SIZE(MuxDesc7),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO7
	{ 49,	NONE,	8,	8,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc8,	MUX_SIZE(MuxDesc8),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO8
	{ 128,	NONE,	2,	2,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc2,	MUX_SIZE(MuxDesc2),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO2
	{ 129,	NONE,	4,	4,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc4,	MUX_SIZE(MuxDesc4),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO4
	{ 130,	NONE,	0,	0,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc0,	MUX_SIZE(MuxDesc0),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO0
	{ 131,	NONE,	1,	1,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc1,	MUX_SIZE(MuxDesc1),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO1
	{ 165,	NONE,	19,	19,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc19,	MUX_SIZE(MuxDesc19),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO19 (AIN5)
	{ 182,	NONE,	6,	6,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc6,	MUX_SIZE(MuxDesc6),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO6
	{ 183,	NONE,	9,	9,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc9,	MUX_SIZE(MuxDesc9),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO9

	{ 200,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for GPIO #44  - Arduino ID IO14
	{ 201,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for GPIO #45  - Arduino ID IO15
	{ 202,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for GPIO #46  - Arduino ID IO16
	{ 203,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for GPIO #47  - Arduino ID IO17
	{ 204,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for GPIO #43  - Arduino ID IO18
	{ 205,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for GPIO #41  - Arduino ID IO19

	{ 208,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #44  - Arduino ID IO14
	{ 209,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #45  - Arduino ID IO15
	{ 210,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #46  - Arduino ID IO16
	{ 211,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #47  - Arduino ID IO17
	{ 212,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #43  - Arduino ID IO18
	{ 213,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #41  - Arduino ID IO19

	{ 214,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Tri-state enable for all pins

	{ 216,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #130 - Arduino ID IO0
	{ 217,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #131 - Arduino ID IO1
	{ 218,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #128 - Arduino ID IO2
	{ 219,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #12  - Arduino ID IO3
	{ 220,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #129 - Arduino ID IO4
	{ 221,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #13  - Arduino ID IO5
	{ 222,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #182 - Arduino ID IO6
	{ 223,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #48  - Arduino ID IO7
	{ 224,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #49  - Arduino ID IO8
	{ 225,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #183 - Arduino ID IO9
	{ 226,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #111 - Arduino ID IO10
	{ 227,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #115 - Arduino ID IO11
	{ 228,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #114 - Arduino ID IO12
	{ 229,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Pullup Control for GPIO #109 - Arduino ID IO13

	{ 232,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #44 - Arduino ID IO14
	{ 233,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #45 - Arduino ID IO15
	{ 234,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #46 - Arduino ID IO16
	{ 235,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #47 - Arduino ID IO17
	{ 236,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #43 - Arduino ID IO18
	{ 237,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #41 - Arduino ID IO19

	{ 248,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #130 - Arduino ID IO0
	{ 249,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #131 - Arduino ID IO1
	{ 250,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #128 - Arduino ID IO2
	{ 251,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #12 -  Arduino ID IO3
	{ 252,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #129 - Arduino ID IO4
	{ 253,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #13 - Arduino ID IO5
	{ 254,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #182 - Arduino ID IO6
	{ 255,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #48 - Arduino ID IO7
	{ 256,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #183 - Arduino ID IO8
	{ 257,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #49 - Arduino ID IO9
	{ 258,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #49 - Arduino ID IO10
	{ 259,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #49 - Arduino ID IO11
	{ 260,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #49 - Arduino ID IO12
	{ 261,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Direction Control for GPIO #49 - Arduino ID IO13

	{ 240,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for Arduino ID IO10
	{ 241,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for Arduino ID IO11
	{ 242,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for Arduino ID IO12
	{ 243,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// PinMux GPIO #40 - Arduino ID IO13
	{ 262,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for Arduino ID IO11
	{ 263,	NONE,	NONE,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL },	// Mux Control for Arduino ID IO10
};



uint32_t sizeof_g_APinDescription;

uint32_t ardPin2DescIdx[GPIO_TOTAL];

// Sorted by Linux PWM ID
PwmDescription g_APwmDescription[] = {
	{ 0,	3,	-1,	-1,	GP12_PWM0,	-1 },
	{ 1,	5,	-1,	-1,	GP13_PWM1,	-1 },
	{ 2,	6,	-1,	-1,	GP182_PWM2,	-1 },
	{ 3,	9,	-1,	-1,	GP183_PWM3,	-1 },
};
uint32_t sizeof_g_APwmDescription;

AdcDescription g_AdcDescription[] = {
	{ 0,	-1 },
	{ 1,	-1 },
	{ 2,	-1 },
	{ 3,	-1 },
	{ 4,	-1 },
	{ 5,	-1 },
};
uint32_t sizeof_g_AdcDescription;

// Sorted Arduino Pin ID
PinState g_APinState[]=
{
	/* uCurrentPwm	uPwmEnabled	uCurrentInput	uCurrentAdc		*/
	{ 0,		0,		1,		0},	/* 0		*/
	{ 0,		0,		1,		0},	/* 1		*/
	{ 0,		0,		1,		0},	/* 2		*/
	{ 0,		0,		1,		0},	/* 3  - PWM	*/
	{ 0,		0,		1,		0},	/* 4 		*/
	{ 0,		0,		1,		0},	/* 5  - PWM 	*/
	{ 0,		0,		1,		0},	/* 6  - PWM	*/
	{ 0,		0,		1,		0},	/* 7 		*/
	{ 0,		0,		1,		0},	/* 8 		*/
	{ 0,		0,		1,		0},	/* 9  - PWM	*/
	{ 0,		0,		1,		0},	/* 10 - PWM	*/
	{ 0,		0,		1,		0},	/* 11 - PMW	*/
	{ 0,		0,		1,		0},	/* 12		*/
	{ 0,		0,		1,		0},	/* 13		*/
	{ 0,		0,		1,		0},	/* 14 - ADC	*/
	{ 0,		0,		1,		0},	/* 15 - ADC	*/
	{ 0,		0,		1,		0},	/* 16 - ADC	*/
	{ 0,		0,		1,		0},	/* 17 - ADC	*/
	{ 0,		0,		1,		0},	/* 18 - ADC	*/
	{ 0,		0,		1,		0},	/* 19 - ADC	*/
};
uint32_t sizeof_g_APinState;

#ifdef __cplusplus
}
#endif


RingBuffer rx_buffer1;
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;

TTYUARTClass Serial(&rx_buffer1, 0, false);	// ttyGS0  (USB serial)
TTYUARTClass Serial1(&rx_buffer2, 1, false);	// ttyQRK0 (IO0/1)


// ----------------------------------------------------------------------------

int variantPinMode(uint8_t pin, uint8_t mode)
{
	/*
	 * Standard (sysfs) or fast-mode UIO options are available for some pins
	 *
	 * The pin at this time is set to Fast-mode by default, if available
	 */

	int ret = 0;
	PinDescription *p = NULL;

	/* Search for entry */
	if ((p = pinDescriptionById(pin)) == NULL) {
		trace_error("%s: invalid pin %u\n", __func__, pin);
		return PIN_EINVAL;
	}

	/* Alternate entries for Fast-Mode GPIO: enable by default if available */
	if (p->pAlternate) {
		p->iAlternate = 1;
		trace_debug("%s: enable Fast-Mode SoC GPIO for pin%u",
			    __func__, pin);
	}
	
	return 0;
}

int variantPinModeIRQ(uint8_t pin, uint8_t mode)
{
	return 0;
}

/*
 * Set the pin as used for PWM and do the muxing at SoC level to enable
 * the PWM output.
 */
void turnOnPWM(uint8_t pin)
{
	int i;

	/* Mark PWM enabled on pin */
	g_APinState[pin].uCurrentPwm = 1;
	g_APinState[pin].uCurrentAdc = 0;

	for (i = 0; i < sizeof_g_APwmDescription; i++)
		if (g_APwmDescription[i].ulArduinoId == pin) {
			sysfsGpioSetCurrentPinmux(g_APwmDescription[i].pwmChPinId,
					PIN_MODE_1);
			break;
		}
}

void turnOffPWM(uint8_t pin)
{
	int handle = 0, ret = 0;
	PinDescription *p = NULL;

	// Scan mappings
	if ((p = pinDescriptionById(pin)) == NULL) {
		trace_error("%s: invalid pin %u\n", __func__, pin);
		return;
	}

	pin2alternate(&p);

	if(p->ulArduinoId == pin) {
		handle = pin2pwmhandle_enable(pin);
		if ((int)PIN_EINVAL == handle) {
			trace_error("%s: bad handle for pin%u",
					__func__, pin);
			return;
		}
		if (sysfsPwmDisable(handle)) {
			trace_error("%s: couldn't disable pwm "
					"on pin%u", __func__, pin);
			return;
		}

		/* Mark PWM disabled on pin */
		g_APinState[pin].uCurrentPwm = 0;
		g_APinState[pin].uPwmEnabled = 0;

		return;
	}

	trace_error("%s: unknown pin%u", __func__, pin);
}

void variantEnableFastGpio(int pin)
{
	int entryno = ardPin2DescIdx[pin];
	PinDescription *p = NULL;
	int ret = 0;

	if (entryno >= sizeof_g_APinDescription) {
		trace_error("%s: ardPin2DescIdx[%d] == %d >= "
				"sizeof_g_APinDescription", __func__, pin, entryno);
		return;
	}

	/* Enable alternate to route to SoC */
	p = &g_APinDescription[entryno];
	p->iAlternate = 1;
}

/*
 * Set the PWM description table accordingly following the PWM swizzler present
 * on the board
 *
 * Suppose the function is called like this: setPwmSwizzler(3, 5, 10, 11). That
 * means the swizzler configuration is as follows:
 *
 *   PWM channel/output 0 is connected to pin 3
 *   PWM channel/output 1 is connected to pin 5
 *   PWM channel/output 2 is connected to pin 10
 *   PWM channel/output 3 is connected to pin 11
 *
 * Default config follows swizzler's default config (3, 5, 6, 9). To change it
 * just call the function in your sketch's setup()
 *
 * Making modifications to the swizzler will break standard GPIO functionality
 * on the affected pins.
 */
void setPwmSwizzler(uint8_t pwmout0, uint8_t pwmout1, uint8_t pwmout2,
		uint8_t pwmout3)
{
	int i;

	/* All parameters must be different */
	if (any_equal(pwmout0, pwmout1, pwmout2, pwmout3)) {
		trace_error("%s All pwm outputs should be different. Got: "
				"%u, %u, %u, %u", __func__, pwmout0, pwmout1,
				pwmout2, pwmout3);
		return;
	}

	/* All parameters must be valid PWM pins */
	if (!all_pwms(pwmout0, pwmout1, pwmout2, pwmout3)) {
		trace_error("%s Some pwm outputs are not valid. Got: "
				"%u, %u, %u, %u", __func__, pwmout0, pwmout1,
				pwmout2, pwmout3);
		return;
	}

	/* Fill the description table with the new pin layout */
	for (i = 0; i < sizeof_g_APwmDescription; i++)
		if (g_APwmDescription[i].ulPWMId == 0)
			g_APwmDescription[i].ulArduinoId = pwmout0;
		else if (g_APwmDescription[i].ulPWMId == 1)
			g_APwmDescription[i].ulArduinoId = pwmout1;
		else if (g_APwmDescription[i].ulPWMId == 2)
			g_APwmDescription[i].ulArduinoId = pwmout2;
		else if (g_APwmDescription[i].ulPWMId == 3)
			g_APwmDescription[i].ulArduinoId = pwmout3;
}

void eepromInit(void)
{
	int fd;
	char buf = 0xff;

	/* Do nothing if file exists already */
	if (access(LINUX_EEPROM, F_OK) == 0)
		return;

	if ((fd = open(LINUX_EEPROM, O_RDWR | O_CREAT, 0660)) < 0) {
		trace_error("%s Can't create EEPROM file: %s", __func__,
				strerror(errno));
		return;
	}

	if (lseek(fd, 0, SEEK_SET)) {
		trace_error("%s Can't lseek in EEPROM file: %s", __func__,
						strerror(errno));
		goto err;
	}

	if (write(fd, &buf, LINUX_EEPROM_SIZE) != LINUX_EEPROM_SIZE)
		trace_error("%s Can't write to EEPROM file: %s", __func__,
				strerror(errno));

	trace_debug("%s Created EEPROM file '%s' of size %u bytes", __func__,
			LINUX_EEPROM, LINUX_EEPROM_SIZE);
err:
	close(fd);
}

void init( int argc, char * argv[] )
{
	if(argc > 1)
		if(Serial.init_tty(argv[1]) != 0)
			return;

	if(Serial1.init_tty(LINUX_SERIAL1_TTY) != 0)
		return;

	sizeof_g_APinDescription = sizeof(g_APinDescription)/sizeof(struct _PinDescription);
	sizeof_g_APinState = sizeof(g_APinState)/sizeof(struct _PinState);
	pinInit();

	/* Initialize fast path to GPIO */
	if (fastGpioPciInit())
		trace_error("Unable to initialize fast GPIO mode!");

	sizeof_g_APwmDescription = sizeof(g_APwmDescription)/sizeof(struct _PwmDescription);
	pwmInit();

	sizeof_g_AdcDescription = sizeof(g_AdcDescription)/sizeof(struct _AdcDescription);
	adcInit();

	eepromInit();
}

