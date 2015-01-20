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

#ifndef Softwareservo_h
#define Softwareservo_h


#include <pthread.h>
#include <Arduino.h>

#define MIN_ANGLE		0	// min angle
#define MAX_ANGLE		180	// max angle
#define MIN_PULSE_WIDTH		544	// the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH		2400	// the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH	1500	// default pulse width when servo is attached

struct servo_thread_param
		{
		  unsigned int pin;
		  unsigned int angle;
		  int freq;
		  int threadID;
		};

class SoftwareServo {
	public:
		SoftwareServo();
		/*
		 * Attach the given pin and sets pinMode
		 * returns channel number or 0 if failure.
		 */
		uint8_t attach(int16_t pin);

		/*
		 * As above but also sets min and max values for writes.
		 */
		uint8_t attach(int pin, int min, int max);

		/*
		 *
		 */
		void detach();

		/*
		 * If value is < 200 its treated as an angle, otherwise as pulse
		 * width in microseconds.
		 */
		void write(int val);

		/*
		 * Write pulse width in microseconds.
		 */
		void writeMicroseconds(int value);

		/*
		 * Returns current pulse width as an angle between 0 and 180
		 * degrees.
		 */
		int read();

		/*
		 * Returns current pulse width in microseconds for this servo
		 * (was read_us() in first release)
		 */
		int readMicroseconds();

		/*
		 * Return true if this servo is attached, otherwise false
		 */
		bool attached();

	private:
	
		int min_pulse;
		int max_pulse;
		int usecs;
		bool isAttached;
		int pin;
		bool is188hz;
		int freq;
		int duty;
		int value;
		int currentThreadID;

		void prepare_pin(uint8_t pin);
		
		void setPeriod(unsigned long int period);
		void setDutyCycle(unsigned int duty_cycle);
		void setFrequency(unsigned int frequency);
		void enablePin(bool enable = true);
		void disablePin();
		int handle_duty;
		int handle_enable;
		
		int threadControl[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		pthread_mutex_t pinMutex[14] = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
			PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
			PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
			PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 
			PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};
		void pulse(int usec, int pin);

		static void* callServoHandler(void *arg) {return ((SoftwareServo *)arg)->servoHandler(); }
		void *servoHandler(void);
		
		
};

#endif