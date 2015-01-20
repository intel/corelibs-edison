/*
 * {% copyright %}
 */

#ifndef __VARIANT_H__
#define __VARIANT_H__

#include <stdint.h>
#include <unistd.h>

#include <AnalogIO.h>
#include <wiring_digital.h>
#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
#include "TTYUART.h"
extern TTYUARTClass Serial;
extern TTYUARTClass Serial1;
#endif

#define LINUX_BOOTLOADER_TTY		"/dev/ttyGS0"
#define LINUX_SERIAL1_TTY		"/dev/ttyMFD1"
#define LINUX_SPIDEV			"/dev/spidev5.1"
#define SYS_BUS_IIO_DEVICES		"/sys/bus/iio/devices"
#define LINUX_ADC_DEVICE_NAME_FMT	(SYS_BUS_IIO_DEVICES "/%s/name")
#define LINUX_ADC_DEVICE_NAME		"ads7955"
#define LINUX_ADC_FMT			(SYS_BUS_IIO_DEVICES "/%s/in_voltage%d_raw")
#define LINUX_I2C_ADAPTER		6

#define LINUX_GPIO_ROOT			"/sys/class/gpio/"
#define LINUX_GPIO_EXPORT		LINUX_GPIO_ROOT "export"
#define LINUX_GPIO_VALUE_FMT		LINUX_GPIO_ROOT "gpio%u/value"
#define LINUX_GPIO_DIRECTION_FMT	LINUX_GPIO_ROOT "gpio%u/direction"
#define LINUX_GPIO_DRIVE_FMT		LINUX_GPIO_ROOT "gpio%u/drive"
#define LINUX_GPIO_EDGE_FMT		LINUX_GPIO_ROOT "gpio%u/edge"
#define LINUX_GPIO_LEVEL_FMT		LINUX_GPIO_ROOT "gpio%u/level"

#define LINUX_GPIO_DEBUG_ROOT		"/sys/kernel/debug/gpio_debug/"
#define LINUX_GPIO_CURRENT_PINMUX_FMT	LINUX_GPIO_DEBUG_ROOT "gpio%u/current_pinmux"

#define LINUX_PWM_ROOT			"/sys/class/pwm/pwmchip0/"
#define LINUX_PWM_EXPORT		LINUX_PWM_ROOT "export"
#define LINUX_PWM_PERIOD_FMT		LINUX_PWM_ROOT "pwm%u/period"
#define LINUX_PWM_DUTY_FMT		LINUX_PWM_ROOT "pwm%u/duty_cycle"
#define LINUX_PWM_ENABLE_FMT		LINUX_PWM_ROOT "pwm%u/enable"

#define PLATFORM_NAME			"GalileoGen2"	// In /sys/devices/platform

#define ADC_RESOLUTION			12
#define PWM_RESOLUTION			8

/*
 * Define period such that the PWM frequency is as close as possible to Arduino
 * Uno's one (~490Hz).
 * The following value gives a frequency of 483Hz (scope capture).  Do not
 * touch unless you know what you're doing.
 */
#define SYSFS_PWM_PERIOD_NS		2048000
#define SYSFS_PWM_PERIOD_MIN_NS		104
#define SYSFS_PWM_PERIOD_MAX_NS		218453000

#define MAX_VARIANT_HPET_FREQ_HZ	1000

#define VARIANT_TRACE_LEVEL TRACE_LEVEL_DEBUG	// default trace level
//#define VARIANT_TRACE_LEVEL TRACE_LEVEL_INFO	// default trace level

/* Mux selector definition */
struct mux_sel {
	uint32_t sel_id;			// GPIOLib ID
	uint32_t sel_val;
};

/* Mux selects (Arduino Pin ID).  */
#define MUX_SEL_NONE			-1
#define MUX_SEL_UART0_RXD		 0
#define MUX_SEL_UART0_TXD		 1
#define MUX_SEL_SPI1_SS_B		10
#define MUX_SEL_SPI1_MOSI		11
#define MUX_SEL_SPI1_MISO		12
#define MUX_SEL_SPI1_SCK		13
#define MUX_SEL_AD7298_VIN0		14
#define MUX_SEL_AD7298_VIN1		15
#define MUX_SEL_AD7298_VIN2		16
#define MUX_SEL_AD7298_VIN3		17
#define MUX_SEL_AD7298_VIN4		18
#define MUX_SEL_AD7298_VIN5		19
/* The I2C lines are wired incorrectly on Fab-B board (SDA should be on IO18) */
#define MUX_SEL_I2C_SDA			19
#define MUX_SEL_I2C_SCL			18

/* Pins table to be instanciated into variant.cpp */

#define MUX_DEPTH_DIGITAL		0x02
#define MUX_DEPTH_ANALOG		0x01
#define MUX_DEPTH_UART			0x02
#define MUX_DEPTH_SPI			0x03
#define MUX_DEPTH_I2C			0x02
#define GPIO_TOTAL			280

// Need to define port offsets for Core Well and Resume Well registers
#define CW_GPIO_LVL			0x08
#define RW_GPIO_LVL			0x28

// Need to define register offsets for MMIO read and write registers
#define MMIO_GPIO_LVL_OUT		0x00
#define MMIO_GPIO_LVL_IN		0x50

/* EEPROM emulation file */
#define LINUX_EEPROM			"/opt/eeprom"
#define LINUX_EEPROM_SIZE		1024

extern PinDescription g_APinDescription[] ;
extern uint32_t sizeof_g_APinDescription;
extern PwmDescription g_APwmDescription[] ;
extern uint32_t sizeof_g_APwmDescription;
extern AdcDescription g_AdcDescription[] ;
extern uint32_t sizeof_g_AdcDescription;
extern uint32_t ardPin2DescIdx[GPIO_TOTAL];
extern PinState g_APinState[] ;
extern uint32_t sizeof_g_APinState;

extern const int mux_sel_analog[NUM_ANALOG_INPUTS];
extern const int mux_sel_uart[NUM_UARTS][MUX_DEPTH_UART];
extern const int mux_sel_spi[NUM_SPI][MUX_DEPTH_SPI];
extern const int mux_sel_i2c[NUM_I2C][MUX_DEPTH_I2C];

int muxSelectPwmPin(uint8_t pin);
int muxSelectAnalogPin(uint8_t pin);
int muxSelectUart(uint8_t interface);
int muxSelectSpi(uint8_t interface);
int muxSelectI2c(uint8_t interface);

const unsigned mapUnoPinToSoC(uint8_t pin);

int variantPinMode(uint8_t pin, uint8_t mode);
int variantPinModeIRQ(uint8_t pin, uint8_t mode);
void turnOffPWM(uint8_t pin);
void turnOnPWM(uint8_t pin);

void variantEnableFastGpio(int pin);

void variantEnablePullup(uint8_t pin, int enable);

void setPwmSwizzler(uint8_t pwm0, uint8_t pwm1, uint8_t pwm2, uint8_t pwm3);

#define fastGpioDigitalWrite(id, val)		fastGpioPciDigitalWrite(id, val)
#define fastGpioDigitalRead(id)			fastGpioPciDigitalRead(id)
#define fastGpioDigitalRegSnapshot(id)		fastGpioPciDigitalRegSnapshot(id)
#define fastGpioDigitalRegWriteUnsafe(id, val)	fastGpioPciDigitalWrite(id, val)

#ifdef __cplusplus
}
#endif


#endif /* __VARIANT_H__ */

