/*
  wiring_digital.c - digital input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis
  Copyright (c) 2014 Intel Corporation

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <trace.h>
#include "Arduino.h"
#include "sysfs.h"
#include "Mux.h"

#define MY_TRACE_PREFIX "wiring_digital"

PinDescription *pinDescriptionById(uint8_t pin)
{
        uint32_t pin_idx = PIN_NOT_FOUND;

        if (unlikely(pin >= GPIO_TOTAL))
                return NULL;

        pin_idx = ardPin2DescIdx[pin];
        if (unlikely(pin_idx == PIN_NOT_FOUND))
                return NULL;

        return &g_APinDescription[pin_idx];
}

static const int pin2gpio(uint8_t pin)
{
	PinDescription *p = NULL;

	if ((p = pinDescriptionById(pin)) == NULL)
		return PIN_EINVAL;

	pin2alternate(&p);

	return p->ulGPIOId;
}

// Just trust the input - this is a requirement for running select(); on gpiolib derived handles
int pinHandleReopen(uint8_t index)
{
	PinDescription *p = &g_APinDescription[index];
	pin2alternate(&p);

	close(p->iHandle);
	p->iHandle = open(p->sPath, O_RDWR);

	return p->iHandle;
}

int pinGetIndex(uint8_t pin)
{
	PinDescription *p = NULL;

	if ((p = pinDescriptionById(pin)) == NULL)
		return PIN_EINVAL;

	return ardPin2DescIdx[p->ulArduinoId];
}

int gpio2gpiohandle(uint32_t gpio)
{
	int i = 0;
	PinDescription *p = NULL;
	for (i = 0; i < sizeof_g_APinDescription; i ++) {
		p = &g_APinDescription[i];
		pin2alternate(&p);
		if (p->ulGPIOId == gpio) {
			return p->iHandle;
		}
	}
	return PIN_EINVAL;
}

int pin2gpiohandle(uint8_t pin)
{
	PinDescription *p = NULL;

	if ((p = pinDescriptionById(pin)) == NULL)
		return PIN_EINVAL;

	pin2alternate(&p);

	return p->iHandle;
}

char * pin2path(uint8_t pin)
{
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL)
		return NULL;

	// Scan mappings
	p = &g_APinDescription[ardPin2DescIdx[pin]];
	pin2alternate(&p);

	return p->sPath;
}

void pinMode(uint8_t pin, uint8_t mode)
{
	int ret = 0;
	uint32_t gpio = 0;
	PinDescription *p = NULL;

	ret = variantPinMode(pin, mode);
	if (ret) {
		trace_error("%s: variantPinMode failed\n", __func__);
		return;
	}

	if ((p = pinDescriptionById(pin)) == NULL) {
		trace_error("%s: invalid pin id: %u\n", __func__, pin);
		return;
	}

	gpio = pin2gpio(pin);
	if (gpio == PIN_EINVAL){
		trace_error("%s: pin %d out of range (gpio%d)", __func__, pin, gpio);
		return;
	}

	turnOffPWM(pin);
	trace_debug("%s: pin=%d, mode=%d", __func__, pin, mode);

	switch (mode) {
	/* TODO this doesn't handle the (input, high) setup yet!!! */
	case INPUT:
		ret = muxSelect(pin, FN_GPIO_INPUT_HIZ);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		g_APinState[pin].uCurrentInput = 1;

		/* Cover alternate gpios too */
		for (; p; p = p->pAlternate) {
			/* Hi-Z  */
			trace_debug("%s: setting gpio%u to input_hiz",
					__func__, p->ulGPIOId);
			sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
			sysfsGpioDirection(p->ulGPIOId, 0, NONE);
		}
		break;
	case INPUT_PULLUP:
		ret = muxSelect(pin, FN_GPIO_INPUT_PULLUP);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		g_APinState[pin].uCurrentInput = 1;

		/* Cover alternate gpios too */
		for (; p; p = p->pAlternate) {
			/* Enable pullup  */
			trace_debug("%s: setting gpio%u to input_pullup",
					__func__, p->ulGPIOId);
			if (p->iExtPullup)
				sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
			else
				sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLUP);
			sysfsGpioDirection(p->ulGPIOId, 0, NONE);
		}
		break;
	case OUTPUT_FAST:
		g_APinState[pin].uCurrentInput = 0;
		trace_debug("%s: setting gpio%u to output_fast",
				__func__, pin2gpio(pin));
		variantEnableFastGpio(pin);
		/* Ensure that alternate pin selection is done BEFORE doing mux config */
		ret = muxSelect(pin, FN_GPIO_OUTPUT);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		/* Refresh pin2gpio */
		sysfsGpioDirection(pin2gpio(pin), 1, 0);
		break;
	case INPUT_FAST:
		g_APinState[pin].uCurrentInput = 1;
		trace_debug("%s: setting gpio%u to input_fast",
				__func__, pin2gpio(pin));
		variantEnableFastGpio(pin);
		/* Ensure that alternate pin selection is done BEFORE doing mux config */
		ret = muxSelect(pin, FN_GPIO_INPUT_HIZ);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		/* Refresh pin2gpio */
		sysfsGpioDirection(pin2gpio(pin), 0, NONE);
		break;
	default:
		ret = muxSelect(pin, FN_GPIO_OUTPUT);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		g_APinState[pin].uCurrentInput = 0;
		trace_debug("%s: setting gpio%u to output",
					__func__, pin2gpio(pin));
		/* Output.  Strong drive mode, default value to 0  */
		sysfsGpioSetDrive(gpio, GPIO_DRIVE_STRONG);
		sysfsGpioDirection(gpio, 1, 0);
		break;
	}

	g_APinState[pin].uCurrentAdc = 0;
}

/**
 * pinModeIRQ
 *
 * pin - arduino pin
 * mode - one of the modes defined for arudino IRQs LOW , CHANGE, RISING, FALLING, HIGH
 */
int pinModeIRQ(uint8_t pin, int8_t mode)
{
	int ret = 0;
	uint32_t gpio = 0;

	ret = variantPinModeIRQ(pin, mode);
	if (ret) {
		trace_error("%s: variantPinMode failed\n", __func__);
		return ret;
	}

	// Configure pin as input
	ret = muxSelect(pin, FN_GPIO_INPUT_HIZ);
	if (ret) {
		trace_error("%s: can't set mux for pin%d\n", __func__, pin);
		return ret;
	}

	gpio = pin2gpio(pin);
	if (gpio == PIN_EINVAL){
		trace_error("%s: pin %d out of range (gpio%d)", __func__, pin, gpio);
		return gpio;
	}

	trace_debug("%s: pin=%d, gpio=%d, mode=%d", __func__, pin, gpio, mode);

	// Now set the mode for IRQ granularity
	switch (mode) {
		case NONE:
		case CHANGE:
		case RISING:
		case FALLING:
			ret = sysfsGpioEdgeConfig(gpio, mode);
			break;
		case LOW:
		case HIGH:
			// Implies edge config none
			ret = sysfsGpioEdgeConfig(gpio, NONE);
			if ( ret < 0 )
				break;

			// Set to active high or active low
			ret = sysfsGpioLevelConfig(gpio, mode);
			break;
		default:
			ret = PIN_EINVAL;
			break;
	}

	return ret;
}

static void digitalWriteSetVal(register uint8_t pin, register uint8_t val)
{
	int ret = 0;
	PinDescription *p = NULL;

	if ((p = pinDescriptionById(pin)) == NULL) {
		trace_error( "%s: invalid pin %u\n", __func__, pin);
		return;
	}
	pin2alternate(&p);

	/* perform screaming fast direct register access */
	if (likely(p->ulFastIOInfo != NONE)) {
		fastGpioDigitalWrite(p->ulFastIOInfo, val);
		return;
	}

	/* else go through sysfs like good citizens */
	if (unlikely(p->iHandle == PIN_EINVAL)){
		trace_error("%s: pin %d out of range (gpio%d)", __func__, pin, pin2gpio(pin));
		return;
	}

	ret = sysfsGpioSet(p->iHandle, (unsigned int)val);
	if (unlikely(ret)) {
		trace_error( "%s: err=%d", __func__, ret);
	}
}

void digitalWrite(register uint8_t pin, register uint8_t val)
{
	PinDescription *p = NULL;

	/* check if the pin id is valid to index g_APinState array */
	if (unlikely(pin >= sizeof_g_APinState)) {
		trace_error("%s: can't index pinState with pin %u\n", __func__, pin);
		return;
	}

	if (unlikely(g_APinState[pin].uCurrentInput)) {
		if (val) {
			trace_debug("%s: input pin%u driven high: enabling "
					"pullup", __func__, pin);
			pinMode(pin, INPUT_PULLUP);
		} else {
			trace_error("%s: input pin%u driven low!", __func__,
					pin);
		}
		return;
	}

	if (unlikely(g_APinState[pin].uCurrentPwm)) {
		turnOffPWM(pin);
		muxSelect(pin, FN_GPIO_OUTPUT);
	}

	if ((p = pinDescriptionById(pin)) == NULL) {
		trace_error( "%s: invalid pin %u\n", __func__, pin);
		return;
	}

	digitalWriteSetVal(pin, val);

	// alias - enable w/o on Fab D for waggle of pin 20 to compensate for pin 13 error
	if (unlikely(p->ulGPIOAlias != NONE))
		digitalWriteSetVal(p->ulGPIOAlias, val);
}

int digitalRead(uint8_t pin)
{
	PinDescription *p = NULL;
	int handle = pin2gpiohandle(pin);

	if (handle == PIN_EINVAL){
		trace_error("%s: pin %d out of range", __func__, pin);
		return -1;
	}

	if (unlikely((p = pinDescriptionById(pin)) == NULL)) {
		trace_error("%s: invalid pin %u\n", __func__, pin);
		return -1;
	}

	pin2alternate(&p);

    if (likely(p->ulFastIOInfo != NONE))
		return fastGpioDigitalRead(p->ulFastIOInfo);

	if (sysfsGpioGet(handle))
		return 1;
	return 0;
}

/*
 * Initialise the GPIO for execution.
 *
 * 1. Export all the Izmir's GPIOs to sysfs
 * 2. Initialise default pin routing on Arduino shield
 * 3. TODO reset pin
 *
 * XXX error reporting
 */
void pinInit(void)
{
	int i = 0;
	PinDescription *p = NULL;
	memset(&ardPin2DescIdx, PIN_NOT_FOUND, sizeof(ardPin2DescIdx));
	/*
	 * Export all GPIOs
	 */
	for (i = 0; i < sizeof_g_APinDescription; i++) {
		for (p = &g_APinDescription[i]; p != NULL; p = p->pAlternate) {
			/* Skip past any system reserved pin */
			if (p->tCurrentType == FN_RESERVED)
				continue;

			p->iHandle = sysfsGpioExport(p->ulGPIOId, p->sPath, sizeof(p->sPath));
			if (p->iHandle < 0){
				trace_error("unable to open GPIO %d", p->ulGPIOId);
				trace_info("unable to open GPIO %d", p->ulGPIOId);
				//assert(0);
			}
		}

		/* Fill entry in uint32_t ardPin2PinDescIdx */
		if (g_APinDescription[i].ulArduinoId != NONE){
			ardPin2DescIdx[g_APinDescription[i].ulArduinoId] = i;
		}
	}

	/* Loop through array and set GPIO to the initial state indicated by the descriptor array */
	for (i = 0; i < sizeof_g_APinDescription; i++) {
		for (p = &g_APinDescription[i]; p != NULL; p = p->pAlternate) {

			if (p->iHandle < 0 && p->tCurrentType != FN_RESERVED) {
				trace_error("%s: invalid iHandle for gpio%u",
						__func__, p->ulGPIOId);
				continue;
			}

			if (p->tCurrentType & FN_SWITCH) {
				/*
				 * GPIOs not exported to Arduino header, used for switching board-level components.
				 * Default drive mode is output with pullup on, unless no output level specified.
				 */
				if (p->ulFixedState == NONE) {
					/* No output, so switch to HiZ input */
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
					sysfsGpioDirection(p->ulGPIOId, 0, NONE);
				} else {
					/* Output defined as LOW or HIGH */
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLUP); /* For Fab-D backward compatibility (TODO - ensure this is ok for Gen2 as well) */
					sysfsGpioDirection(p->ulGPIOId, 1, p->ulFixedState);
				}
			} else if (p->tCurrentType & FN_GPIO) {
				/*
				 * GPIOs exported on Arduino headers are by default input with no pullup unless specified
				 */
				switch (p->ulFixedState) {
				case HIGH:
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLUP);
					break;
				case LOW:
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLDOWN);
					break;
				default:
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
				}
				sysfsGpioDirection(p->ulGPIOId, 0, NONE);
			} else {
				trace_info("%s gpiolib %d is not a candidate for direction confiugration", __func__, p->ulGPIOId);
			}
		}
	}

	/* Now set the inital mux  */
	muxInit();
}

