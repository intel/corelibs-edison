/*
 * {% copyright %}
 */

#ifndef __MUX_H__
#define __MUX_H__

#include <Arduino.h>		// Contains types
#include <wiring_digital.h>

#ifdef __cplusplus
extern "C" {
#endif

int muxInit(void);
int muxSelect(uint8_t arduino_pin, uint32_t tFunction);

/* Wrapper functions for muxSelect() */
int muxSelectAnalogPin(uint8_t pin);
int muxSelectPwmPin(uint8_t pin);
int muxSelectUart(uint8_t interface);
int muxSelectSpi(uint8_t interface);
int muxSelectI2c(uint8_t interface);


void pin2alternate(PinDescription **p_ptr);

#ifdef __cplusplus
}
#endif

#endif /* __MUX_H__ */

