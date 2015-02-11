/*
 Copyright (c) 2011 Arduino.  All right reserved.
 Copyright (c) 2013 Intel Corporation.

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
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <trace.h>

#include "Arduino.h"
#include "pins_arduino.h"

#define MY_TRACE_PREFIX "wiring_analog"

/* By default, analogRead() should assume a 10-bit ADC resolution
 * However, this can be overriden using analogReadResolution()
 * The Kips Bay ADC supports a 12-bit resolution.
 * */
static uint32_t _readResolution = 10;
static uint32_t _writeResolution = 8;
static uint8_t analog_reference = DEFAULT;

static const int pin2pwm(uint8_t pin)
{
	uint32_t i;

	// Scan mappings
	for (i = 0; i < sizeof_g_APwmDescription; i++){
		if(g_APwmDescription[i].ulArduinoId == pin)
			return g_APwmDescription[i].ulPWMId;
	}

	// Indicate error
	return PIN_EINVAL;
}

int pin2pwmhandle_enable(uint8_t pin)
{
	uint32_t i;

	// Scan mappings
	for (i = 0; i < sizeof_g_APwmDescription; i++){
		if(g_APwmDescription[i].ulArduinoId == pin)
			return g_APwmDescription[i].iHandleEnable;
	}

	// Indicate error
	return PIN_EINVAL;
}

int pin2pwmhandle_duty(uint8_t pin)
{
	uint32_t i;

	// Scan mappings
	for (i = 0; i < sizeof_g_APwmDescription; i++){
		if(g_APwmDescription[i].ulArduinoId == pin)
			return g_APwmDescription[i].iHandleDuty;
	}

	// Indicate error
	return PIN_EINVAL;
}

int pin2pwmhandle_period(uint8_t pin)
{
	uint32_t i;

	// Scan mappings
	for (i = 0; i < sizeof_g_APwmDescription; i++){
		if(g_APwmDescription[i].ulArduinoId == pin)
			return g_APwmDescription[i].iHandlePeriod;
	}

	// Indicate error
	return PIN_EINVAL;
}

void analogReadResolution(uint32_t res)
{
	if (res >= 1 && res <= 32)
		_readResolution = res;
}

void analogWriteResolution(int res)
{
	_writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

void analogReference(uint8_t mode)
{
	/* Retained for backward compatibilty, only DEFAULT is supported */
}

uint32_t analogRead(uint32_t ulPin)
{
	int ret = 0;
	uint8_t adc;
	uint32_t ulValue;
	
	if (ulPin < NUM_ANALOG_INPUTS)
		adc = ulPin;
	else
		adc = ulPin - A0;

	if (adc > NUM_ANALOG_INPUTS) {
		trace_error("Invalid analog input channel specified\n");
		return 0;
	}

	if (unlikely(!g_APinState[A0+adc].uCurrentAdc))
	{
		muxSelectAnalogPin(adc);
		if (ret) {
			trace_error("%s: can't set mux for pin %d\n",
				    __func__, A0+adc);
			return ret;
		}
		g_APinState[A0+adc].uCurrentAdc = 1;
	}

	ulValue = sysfsAdcGet(g_AdcDescription[adc].iHandle);

	/* Kips Bay has a 12-bit ADC. Adjust for desired resolution */
	if (ulValue)
		ulValue = mapResolution(ulValue, ADC_RESOLUTION,
					_readResolution);

	return ulValue;
}

void analogWrite(uint32_t ulPin, uint32_t ulValue)
{
	int ret = 0;

	trace_debug("%s ulPin=%u ulValue=%u", __func__, ulPin, ulValue);

	if (!digitalPinHasPWM(ulPin)) {
		trace_error("%s pin%u has no pwm", __func__, ulPin);
		return;
	}

	if (!g_APinState[ulPin].uCurrentPwm) {
		trace_debug("%s: turning on pwm%u", __func__, ulPin);
		ret = muxSelectPwmPin(ulPin);
		if (ret) {
			trace_error("%s: can't set mux for pin%d",
				    __func__, ulPin);
			return;
		}
		turnOnPWM(ulPin);
	}

	if (ulValue) {
		ulValue = mapResolution(ulValue, _writeResolution,
				PWM_RESOLUTION);
				
		if (!g_APinState[ulPin].uPwmEnabled) {
			if (sysfsPwmEnable(pin2pwmhandle_enable(ulPin)) < 0) {
				trace_error("%s Failed enabling PWM on pin %u",
					__func__, ulPin);
				return;
			}
		g_APinState[ulPin].uPwmEnabled = 1;
		}

	if (sysfsPwmSetDutyCycle(pin2pwmhandle_duty(ulPin), ulValue))
		trace_error("%s Failed setting duty_cycle: %u", __func__,
				ulValue);
				
	}
	else {
		PinDescription *p = NULL;

		if ((p = pinDescriptionById(ulPin)) == NULL) {
			//trace_error( "%s: invalid pin %u\n", __func__, pin);
			return;
		}
		pin2alternate(&p);
		int pwmGpio = p->ulGPIOId;
		char cmd[40];
		
		memset(cmd, '\0', sizeof(cmd));
		sprintf(cmd, "echo %d %s", pwmGpio, "> /sys/class/gpio/unexport");
		system(cmd);
		digitalWrite(ulPin, 0);	//workaround for tangier SoC bug when setting duty cycle to 0%
		memset(cmd, '\0', sizeof(cmd));
		sprintf(cmd, "echo %d %s", pwmGpio, "> /sys/class/gpio/export");
		system(cmd);
	}

	
}

void pwmInit(void)
{
	int i = 0;
	int ret = 0;
	PwmDescription *pwm;

	for (i = 0; i < sizeof_g_APwmDescription; i++) {
		pwm = &g_APwmDescription[i];
		if (sysfsPwmExport(pwm->ulPWMId, &pwm->iHandleEnable,
				&pwm->iHandleDuty, &pwm->iHandlePeriod))
			continue;

		/* If for some reason the duty cycle on this channel was bigger
		 * than SYSFS_PWM_PERIOD_NS setting the period would fail.
		 * Set the duty_cycle to 0 first avoids this possible issue. */
		if (sysfsPwmSetDutyCycle(pwm->iHandleDuty, 0))
			continue;

		if (sysfsPwmSetPeriod(pwm->iHandlePeriod, SYSFS_PWM_PERIOD_NS))
			continue;

		if (sysfsPwmDisable(pwm->iHandleEnable))
			continue;

		trace_debug("%s pwm=%u, handleEnable=%p, handlePeriod=%p, handleDuty=%p",
				__func__, pwm->ulPWMId, pwm->iHandleEnable,
				pwm->iHandlePeriod, pwm->iHandleDuty);
	}
}

void adcInit(void)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < sizeof_g_AdcDescription; i++) {
		ret = sysfsAdcExport(i, &g_AdcDescription[i].iHandle);
		if (ret < 0) {
			trace_error("unable to open adc %d", i);
		}

		trace_debug("adc=%d iHandle=%d",
			    i, g_AdcDescription[i].iHandle);
	}

	/* No need to set any muxing: done by pinMode.  */

	return 0;
}

#ifdef __cplusplus
}
#endif
