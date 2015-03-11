/*
 * Copyright (c) 2014 Intel Corporation
 * Dino Tinitigan <dino.tinitigan@intel.com>
 *
 * Arduino SoftwareServo library for Intel Edison
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 * Bit-bangs servo signals for Intel Edison
*/

#include "SoftwareServo.h"
#include <pthread.h>

SoftwareServo::SoftwareServo()
{
	this->min_pulse = MIN_PULSE_WIDTH;
	this->max_pulse = MAX_PULSE_WIDTH;
	this->freq = 50;
}

void SoftwareServo::enablePin(bool enable)
{
	//set the mux and open the handles
	pinMode(pin, OUTPUT);
}

void SoftwareServo::disablePin()
{
	pinMode(pin, INPUT);
}

void SoftwareServo::setPeriod(unsigned long int period)
{

}

void SoftwareServo::setDutyCycle(unsigned int duty_cycle)
{

}

void SoftwareServo::prepare_pin(uint8_t pin)
{
	this->enablePin();
}

void SoftwareServo::detach()
{
	if (this->isAttached) {
		this->isAttached = false;
	}
	this->disablePin();

}

uint8_t SoftwareServo::attach(int16_t pin)
{
	return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t SoftwareServo::attach(int pin, int min, int max)
{

	// let's check the boundaries
	if (min < MIN_PULSE_WIDTH)
		min = MIN_PULSE_WIDTH;
	if (max > MAX_PULSE_WIDTH)
		max = MAX_PULSE_WIDTH;

	this->pin = pin;
	this->min_pulse = min;
	this->max_pulse = max;
	if(pin <= 13)
	{
		this->isAttached = true;
		//this->setPeriod(PWM_50Hz);
		//this->setDutyCycle(DEFAULT_PULSE_WIDTH * 1000);
		this->enablePin();
	}
	else
	{
		this->isAttached = false;
		this->disablePin();
		
	}
	return pin;
}

void SoftwareServo::writeMicroseconds(int microsecs)
{
	if(this->isAttached)
	{
		// checking the boundaries
		if (microsecs < this->min_pulse)
			microsecs = this->min_pulse;
		if (microsecs > this->max_pulse)
			microsecs = this->max_pulse;

		
		this->setDutyCycle(microsecs * 1000);

		// update last microseconds passed
		this->usecs = microsecs;
		
		
		pthread_mutex_lock(&pinMutex[this->pin]);
		int tempThreadControl = ++threadControl[this->pin];
		pthread_mutex_unlock(&pinMutex[this->pin]);
	  
		pthread_t servoThread;
		
		this->value = microsecs;
		this->currentThreadID = tempThreadControl;
		
		int iret1 = pthread_create( &servoThread, NULL, &SoftwareServo::callServoHandler, this);
		pthread_detach(servoThread);
	}

}

void SoftwareServo::write(int val)
{
	// according to Arduino reference lib, if this angle will
	// be bigger than 200, it should be considered as microsenconds
	if(this->isAttached)
	{
		pthread_mutex_lock(&pinMutex[this->pin]);
		int tempThreadControl = ++threadControl[this->pin];
		pthread_mutex_unlock(&pinMutex[this->pin]);
	  
		pthread_t servoThread;
		
		this->value = val;
		this->currentThreadID = tempThreadControl;
		
		int iret1 = pthread_create( &servoThread, NULL, &SoftwareServo::callServoHandler, this);
		pthread_detach(servoThread);
	}
}

int SoftwareServo::read()
{
	return map(this->usecs, this->min_pulse, this->max_pulse, MIN_ANGLE, MAX_ANGLE);
}

int SoftwareServo::readMicroseconds()
{
	return this->usecs;
}

bool SoftwareServo::attached()
{
	return this->isAttached;
}


void* SoftwareServo::servoHandler(void)
{
  int value = this->value;
  int pin = this->pin;
  int freq = this->freq;
  int currentThreadControlID = this->currentThreadID;
  
  while(true)
  {
    if(currentThreadControlID != this->threadControl[pin])
    {
      break;
    }
    
    unsigned long a = micros();
    unsigned long b = a;
    if(value > 180)
    {
      //write microseconds 
      pulse(value, pin);
    }
    else
    {
      int angle = map(value, 0, 180, this->min_pulse, this->max_pulse);
      pulse(angle, pin);
    }
	int period = (1.0/((double)(this->freq)))*1000000;
    while((b-a) < (period))
    {
      b = micros();
    }
  }
  
}

void SoftwareServo::pulse(int usec, int pin)
{
  noInterrupts();
  unsigned long a = micros();
  unsigned long b = a;
  fastDigitalWrite(pin, HIGH);
  while((b-a) < (usec))
  {
    b = micros();
  }
  fastDigitalWrite(pin, LOW); 
  interrupts();
}