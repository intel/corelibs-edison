#ifndef Servo_h
#define Servo_h
#include "variant.h"

// checking if the communication is done using sysfs or cypress
#if PLATFORM_ID == 0x06 
#define SERVO_PWM_WITH_I2C           // CYPRESS IC
#else
#define SERVO_PWM_WITH_SYSFS         // PCA9685 IC        
#endif

#ifdef SERVO_PWM_WITH_SYSFS
#include <interrupt.h>
#include <sysfs.h>
#else
#include "Wire.h" // for cypress (I2C)
#endif

#include "Arduino.h"
#include <stdio.h>

#define MIN_ANGLE		0	// min angle
#define MAX_ANGLE		180	// max angle
#define MIN_PULSE_WIDTH		544	// the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH		2400	// the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH	1500	// default pulse width when servo is attached
#define INVALID_SERVO		255
#define MAX_NUMBER_OF_SERVOS	6	// same number of pins in PWM
#define MY_TRACE_PREFIX		"ServoEdisonLib"
#define PWM_50Hz		19000000


#ifdef SERVO_PWM_WITH_I2C
// Definitions for FabE and FabG

/* How the pins are connected to cypress in FabD */
#define CYPRESS_I2C_ADDRESS 0x20  // I2C address

#else

// Definitions for FabE and FabG
// this function pointer is to avoid the conflict of write C ANSI call
// compared to the Servo method called "write"
static int (*pointer_write)(int handle, const void *buffer,
		unsigned int nbyte) = write;

#endif // PLATFORM_NAME checking

typedef struct {
	uint8_t pin;
	/* true if this channel is enabled, pin not pulsed if false */
	bool isActive;
} servoPinData_t;

class Servo {
	public:
		Servo();
		/*
		 * Attach the given pin to the next free channel, sets pinMode,
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

		/* minimum is this value times 4 added to MIN_PULSE_WIDTH */
		int min;
		/* maximum is this value times 4 added to MAX_PULSE_WIDTH */
		int max;
		uint8_t index;
		int usecs;
		bool isAttached;
		byte pin;
		bool is188hz;
		/* To avoid jitter caused by analogWrite() */
		int lastByteInDuty;

		servoPinData_t pinData[MAX_NUMBER_OF_SERVOS] = {
			{ 3,  false },
			{ 5,  false },
			{ 6,  false },
			{ 9,  false },
			{ 10, false },
			{ 11, false }
		};

		static uint8_t counter;

		void prepare_pin(uint8_t pin);

#ifdef SERVO_PWM_WITH_I2C
		byte transform_cypress_duty_cycle_byte(int microsecs);
		/*
		 * Forces cypress to work in 47.8 hertz
		 */
		void set48hz();

		/*
		 * Forces cypress to work in 188 hertz (better angle resolution)
		 */
		void set188hz();
#else
		void setPeriod(unsigned long int period);
		void setDutyCycle(unsigned int duty_cycle);
		void enablePin(bool enable = true);
		void disablePin();
		int handle_duty;
		int handle_enable;
#endif
};

#endif
