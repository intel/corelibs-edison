/*
sysfs.c
Copyright (c) 2014 Intel Corporation
Copyright (c) 2013 Anuj Deshpande
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General
Public License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330,
Boston, MA 02111-1307 USA
*/

#include <Arduino.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <trace.h>
#include <interrupt.h>
#include <errno.h>
#include <dirent.h>

#define MY_TRACE_PREFIX "sysfs"

#define SYSFS_BUF		0x50

#define WIRE_PWM_DUTY_MAX	((1 << PWM_RESOLUTION) - 1)

static unsigned long current_period = SYSFS_PWM_PERIOD_NS;

static int sysfsPwmOpenHandler(int *handler, const char *path, unsigned pwm)
{
	char fs_path[SYSFS_BUF] = { 0 };
	int fd;

	snprintf(fs_path, sizeof(fs_path), path, pwm);
	if ((fd = open(fs_path,  O_RDWR)) < 0) {
		trace_error("%s Can't open handle to %s: %s", __func__, fs_path,
				strerror(errno));
		return -1;
	}

	*handler = fd;
	return 0;
}

/*
 * Initialize PWMs.
 *  - export PWMs to sysfs
 *  - save persistent handles to enable/duty_cycle/period
 *
 * This is way more coarse-grained than the GPIO version: Arduino code doesn't
 * require too much flexibility.
 */
int sysfsPwmExport(unsigned pwm, int *handle_enable, int *handle_duty,
		int *handle_period)
{
	int fd;
	char export_value[16] = "";

	trace_debug("%s: pwm=%u", __func__, pwm);

	if ((fd = open(LINUX_PWM_EXPORT, O_WRONLY)) < 0) {
		trace_error("%s Can't open handle to %s: %s", __func__,
				LINUX_PWM_EXPORT, strerror(errno));
		return -1;
	}
	lseek(fd, 0, SEEK_SET);

	snprintf(export_value, sizeof(export_value), "%u", pwm);
	write(fd, &export_value, sizeof(export_value));
	/* If PWM channel has been already exported write will fail.
	 * Ignore write return value */
	close(fd);

	/* Open persistent handle to pwm/period */
	if (sysfsPwmOpenHandler(handle_period, LINUX_PWM_PERIOD_FMT, pwm) < 0)
		return -1;

	/* Open persistent handle to pwm/enable */
	if (sysfsPwmOpenHandler(handle_enable, LINUX_PWM_ENABLE_FMT, pwm) < 0)
		return -1;

	/* Open persistent handle to pwm/duty_cycle */
	if (sysfsPwmOpenHandler(handle_duty, LINUX_PWM_DUTY_FMT, pwm) < 0)
		return -1;

	trace_debug("%s: pwm=%u, handle_enable=%d, handle_duty=%d, handle_period=%d",
		    __func__, pwm, *handle_enable, *handle_duty, *handle_period);
	return 0;
}

static inline int setPwmChannelState(int handle_enable, char enable)
{
	int ret = 0;

	lseek(handle_enable, 0, SEEK_SET);
	ret = write(handle_enable, &enable, sizeof(enable));
	if (sizeof(enable) != ret) {
		trace_error("%s Can't write to enable: %s", __func__,
				strerror(errno));
		return -1;
	}
	return 0;
}

int sysfsPwmEnable(int handle_enable)
{
	trace_debug("%s: handle_enable=%p", __func__, handle_enable);
	return setPwmChannelState(handle_enable, '1');
}

int sysfsPwmDisable(int handle_enable)
{
	trace_debug("%s: handle_enable=%p", __func__, handle_enable);
	return setPwmChannelState(handle_enable, '0');
}

int sysfsPwmSetPeriod(int handle_period, unsigned long period)
{
	char value[16] = { 0 };
	int ret;

	if (period < SYSFS_PWM_PERIOD_MIN_NS || period > SYSFS_PWM_PERIOD_MAX_NS) {
		trace_error("%s period must be in range %u:%u", __func__,
				SYSFS_PWM_PERIOD_MIN_NS,SYSFS_PWM_PERIOD_MAX_NS);
		return -1;
	}

	/* get a copy of the current period to use it in sysfsPwmSetDutyCycle */
	current_period = period;

	snprintf(value, sizeof(value), "%lu", current_period);
	lseek(handle_period, 0, SEEK_SET);
	ret = write(handle_period, &value, sizeof(value));
	if (ret != sizeof(value)) {
		trace_error("%s Can't write to period: %s", __func__,
				strerror(errno));
		return -1;
	}
	return 0;
}

int sysfsPwmSetDutyCycle(int handle_duty, unsigned int duty_cycle)
{
	unsigned long long value_duty;

	if (duty_cycle > WIRE_PWM_DUTY_MAX) {
		trace_error("%s duty_cycle must be less than %u", __func__,
				WIRE_PWM_DUTY_MAX);
		return -1;
	}

	value_duty = ((unsigned long long)duty_cycle * current_period) / WIRE_PWM_DUTY_MAX;
	return sysfsPwmSetRawDutyCycle(handle_duty, value_duty);
}

int sysfsPwmSetRawDutyCycle(int handle_duty, unsigned int duty_cycle_ns)
{
	char value[16] = { 0 };
	int ret;

	snprintf(value, sizeof(value), "%u", duty_cycle_ns);
	lseek(handle_duty, 0, SEEK_SET);
	ret = write(handle_duty, &value, sizeof(value));
	if (sizeof(value) != ret) {
		trace_error("%s Can't write to duty_cycle: %s", __func__,
				strerror(errno));
		return -1;
	}

	return 0;
}

static int find_iio_dev_name(char *iio_dev_name, size_t iio_dev_name_len)
{
	DIR *dir;
	struct dirent *iio_dev_dir;
	char adc_dev_name[32];
	char fs_path[SYSFS_BUF];
	int fd;

	if ((dir = opendir(SYS_BUS_IIO_DEVICES)) == NULL) {
		trace_error("%s Can't open directory %s: %s", __func__,
				SYS_BUS_IIO_DEVICES, strerror(errno));
		return -1;
	}

	/* Go through all devices and read their names.
	 * Stop when ads7955 is found. */
	while ((iio_dev_dir = readdir(dir)) != NULL) {
		snprintf(fs_path, sizeof(fs_path), LINUX_ADC_DEVICE_NAME_FMT,
				iio_dev_dir->d_name);

		if ((fd = open(fs_path, O_RDONLY)) < 0)
			continue;

		if (read(fd, adc_dev_name, strlen(LINUX_ADC_DEVICE_NAME)) > 0) {
			/* Make sure strncmp is happy */
			adc_dev_name[sizeof(LINUX_ADC_DEVICE_NAME)] = '\0';

			if (!strncmp(adc_dev_name, LINUX_ADC_DEVICE_NAME,
					(size_t)strlen(LINUX_ADC_DEVICE_NAME)))
				break;
		}
		else {
			trace_error("%s Can't read %s: %s", __func__, fs_path,
					strerror(errno));
		}
		close(fd);
	}
	closedir(dir);

	if (iio_dev_dir == NULL) {
		trace_error("%s Can't find ADC driver '%s' entry in %s", __func__,
				LINUX_ADC_DEVICE_NAME, SYS_BUS_IIO_DEVICES);
		return -1;
	}

	strncpy(iio_dev_name, iio_dev_dir->d_name, iio_dev_name_len);

	trace_debug("%s Found IIO device called %s", __func__, iio_dev_name);

	return 0;
}
/*
 * Initialise ADCs.
 *  - save persistent handles to read ADC input values
 */
int sysfsAdcExport(unsigned adc, int *handle)
{
	char fs_path[SYSFS_BUF];
	static char iio_dev_name[32] = { 0 };
	int fd;

	trace_debug("%s: adc=%u", __func__, adc);

	if (adc > (NUM_ANALOG_INPUTS - 1)) {
		trace_error("%s err adc > %d", __func__, NUM_ANALOG_INPUTS);
		return -1;
	}

	/* We only check for the iio_dev_name once */
	if (iio_dev_name[0] == 0
			&& find_iio_dev_name(iio_dev_name, sizeof(iio_dev_name)) < 0)
		return -1;

	/* Open persistent handle to adc channel */
	snprintf(fs_path, sizeof(fs_path), LINUX_ADC_FMT, iio_dev_name, adc);
	if ((fd = open(fs_path, O_RDONLY)) < 0) {
		trace_error("%s Can't open handle to analog input channel %u: %s",
			    __func__, adc, strerror(errno));
		return -1;
	}
	*handle = fd;

	trace_debug("%s adc=%u, handle=%d, path=%s", __func__, adc, *handle,
			fs_path);

	return 0;
}

uint32_t sysfsAdcGet(int handle)
{
	char strValue[8] = "0";
	int ret = 0;

	lseek(handle, 0, SEEK_SET);
	ret = read(handle, strValue, sizeof(strValue));
	if (unlikely(ret < 0))
	{
		trace_error("Can't read from analog input channel\n");
		return 0;
	}
	strValue[ret] = '\0';

	return atoi(strValue);
}

int sysfsGpioSet(int handle, unsigned int value)
{
	char set_value = 0;

	set_value = '0' + (value ? 1 : 0);
	lseek(handle, 0, SEEK_SET);

	/* Return 0 if success */
	return write(handle, &set_value, 1) != 1;
}

int sysfsGpioGet(int handle)
{
	char get_value = 0;

	lseek(handle, 0, SEEK_SET);
	read(handle, &get_value, 1);
	return (get_value == '1');
}

/* echo 'gpio' > /sys/class/gpio/export */
int sysfsGpioExport(unsigned int gpio, char *path, unsigned int max_path)
{
	FILE *fp = NULL;
	int ret = 0;
	char export_value[4] = "";
	char fs_path[SYSFS_BUF] = LINUX_GPIO_EXPORT;

	trace_debug("%s: gpio%u", __func__, gpio);

	if (path == NULL || max_path < sizeof(fs_path)){
		trace_error("gpio %d max_path %d -EINVAL", gpio, max_path);
		return -1;
	}

	if (gpio > 999) {
		trace_error("cannot handle gpio numbers greater than 999");
		return -1;
	}

	if (NULL == (fp = fopen(fs_path, "ab"))) { //XXX why not rb+?
		trace_error("err export fs_path=%s", fs_path);
		return -1;
	}
	rewind(fp);

	snprintf(export_value, sizeof(export_value), "%u", gpio);
	fwrite(&export_value, sizeof(char), sizeof(export_value), fp);
	fclose(fp);

	/* Open persistent handle to GPIO */
	memset(path, 0x00, max_path);
	snprintf(path, max_path, LINUX_GPIO_VALUE_FMT, gpio);
	ret = open(path, O_RDWR);

	return ret;
}

/* echo 'gpio' > /sys/class/gpio/unexport */
int sysfsGpioUnexport(unsigned int gpio, char *path, unsigned int max_path)
{
	FILE *fp = NULL;
	int ret = 0;
	char export_value[4] = "";
	char fs_path[SYSFS_BUF] = LINUX_GPIO_UNEXPORT;

	trace_debug("%s: gpio%u", __func__, gpio);

	if (path == NULL || max_path < sizeof(fs_path)){
		trace_error("gpio %d max_path %d -EINVAL", gpio, max_path);
		return -1;
	}

	if (gpio > 999) {
		trace_error("cannot handle gpio numbers greater than 999");
		return -1;
	}

	if (NULL == (fp = fopen(fs_path, "ab"))) { //XXX why not rb+?
		trace_error("err export fs_path=%s", fs_path);
		return -1;
	}
	rewind(fp);

	snprintf(export_value, sizeof(export_value), "%u", gpio);
	fwrite(&export_value, sizeof(char), sizeof(export_value), fp);
	fclose(fp);

	/* Open persistent handle to GPIO */
	memset(path, 0x00, max_path);
	snprintf(path, max_path, LINUX_GPIO_VALUE_FMT, gpio);
	ret = open(path, O_RDWR);

	return ret;
}

/* echo 'output' > /sys/class/gpio/gpio'gpio'/direction */
int sysfsGpioDirection(unsigned int gpio, int output, int outval)
{
	FILE *fp = NULL;
	int ret = 0;
	int handle = PIN_EINVAL;
	char dir_value[5] = "";
	char fs_path[SYSFS_BUF] = "";

	trace_debug("%s: gpio%u, output=%d, outval=%d", __func__, gpio, output, outval);

	/* Set GPIO direction  */
	snprintf(fs_path, sizeof(fs_path), LINUX_GPIO_DIRECTION_FMT, gpio);
	if (0 == access(fs_path, F_OK)) { /* Some GPIOs are output-only so check for direction attribute */
		if (NULL == (fp = fopen(fs_path, "rb+"))) {
			trace_error("err direction fs_path=%s\n", fs_path);
			return -1;
		}
		rewind(fp);
		if (output) {
			strcpy(dir_value, outval ? "high" : "low");
		} else {
			strcpy(dir_value, "in");
		}
		fwrite(&dir_value, sizeof(char), sizeof(dir_value), fp);
		fclose(fp);
	}

	/*
	 * Workaround for RTC #55197: some GPIO IPs/drivers won't apply a value
	 * before switching direction to output.
	 * Also handles the fall-through case for output-only GPIOs with no direction attribute
	 */
	if (output) {
		handle = gpio2gpiohandle(gpio);
		if (PIN_EINVAL == handle) {
			ret = handle;
			goto end;
		}
		ret = sysfsGpioSet(handle, outval);
	}

end:
	return ret;
}

int sysfsGpioSetDrive(unsigned int gpio, unsigned int mode)
{
	FILE *fp = NULL;
	int ret = 0;
	char value[8] = "";
	char fs_path[SYSFS_BUF] = "";

	trace_debug("%s: gpio%u, mode=%u", __func__, gpio, mode);

	/* Set GPIO direction  */
	snprintf(fs_path, sizeof(fs_path), LINUX_GPIO_DRIVE_FMT, gpio);
	if (NULL == (fp = fopen(fs_path, "rb+"))) {
		trace_error("err set drive fs_path=%s\n", fs_path);
		return -1;
	}
	rewind(fp);

	switch(mode) {
	case GPIO_DRIVE_PULLUP:
		strcpy(value, "pullup");
		break;
	case GPIO_DRIVE_PULLDOWN:
		strcpy(value, "pulldown");
		break;
	case GPIO_DRIVE_STRONG:
		strcpy(value, "strong");
		break;
	case GPIO_DRIVE_HIZ:
		strcpy(value, "hiz");
		break;
	default:
		trace_error("%s: unknown mode %u", __func__, mode);
		return -1;
		break;
	}

	fwrite(&value, sizeof(char), sizeof(value), fp);
	fclose(fp);

	return ret;
}

int sysfsGpioSetCurrentPinmux(unsigned int gpio, unsigned int mode)
{
	FILE *fp = NULL;
	int ret = 0;
	char value[8] = "";
	char fs_path[SYSFS_BUF] = "";

	trace_debug("%s: gpio%u, mode=%u", __func__, gpio, mode);
	trace_error("%s: gpio%u, mode=%u", __func__, gpio, mode);

	/* Set pinmux mode  */
	snprintf(fs_path, sizeof(fs_path), LINUX_GPIO_CURRENT_PINMUX_FMT, gpio);
	if (NULL == (fp = fopen(fs_path, "rb+"))) {
		trace_error("err set pinmux fs_path=%s\n", fs_path);
		return -1;
	}
	rewind(fp);

	switch(mode) {
	case PIN_MODE_0:
		trace_error("set pinmux mode0 fs_path=%s\n", fs_path);
		strcpy(value, "mode0");
		break;
	case PIN_MODE_1:
		strcpy(value, "mode1");
		break;
	case PIN_MODE_2:
		strcpy(value, "mode2");
		break;
	case PIN_MODE_3:
		strcpy(value, "mode3");
		break;
	case PIN_MODE_4:
		strcpy(value, "mode4");
		break;
	case PIN_MODE_5:
		strcpy(value, "mode5");
		break;
	default:
		trace_error("%s: unknown mode %u", __func__, mode);
		return -1;
		break;
	}

	fwrite(&value, sizeof(char), sizeof(value), fp);
	fclose(fp);

	return ret;
}

/* echo 'output' > /sys/class/gpio/gpio'gpio'/edge */
// mode - one of the modes defined for arudino IRQs CHANGE "both", RISING "rising", FALLING "falling", NONE "none"
int sysfsGpioEdgeConfig(unsigned int gpio, int mode)
{
	FILE *fp = NULL;
	int idx;
	char * edge_state [] = {
		"both",
		"rising",
		"falling",
		"none",
	};
	char fs_path[SYSFS_BUF] = "";

	switch(mode){
		case CHANGE:
			idx = 0;
			break;
		case RISING:
			idx = 1;
			break;
		case FALLING:
			idx = 2;
			break;
		case NONE:
			idx = 3;
			break;
		default:
			return -1;
	}

	trace_debug("%s: gpio%u, edge=%s", __func__, gpio, edge_state[idx]);

	/* Set GPIO direction  */
	snprintf(fs_path, sizeof(fs_path), LINUX_GPIO_EDGE_FMT, gpio);
	if (NULL == (fp = fopen(fs_path, "rb+"))) {
		trace_error("%s err edge fs_path=%s gpio not capable of edge config\n",
				__func__, fs_path);
		return -1;
	}
	rewind(fp);

	fwrite(edge_state[idx], sizeof(char), strlen(edge_state[idx]), fp);
	fclose(fp);

	return 0;
}

/* echo 'output' > /sys/class/gpio/gpio'gpio'/edge */
// mode - one of the modes defined for arudino IRQs HIGH "high", LOW "low"
int sysfsGpioLevelConfig(unsigned int gpio, int mode)
{
	FILE *fp = NULL;
	int ret = 0, idx;
	char * level_state [] = {
		"high",
		"low",
	};
	char fs_path[SYSFS_BUF] = "";

	switch(mode){
		case HIGH:
			idx = 0;
			break;
		case LOW:
			idx = 1;
			break;
		default:
			return -1;
	}

	trace_debug("%s: gpio%u, level_state=%s", __func__, gpio, level_state[idx]);

	/* Set GPIO direction  */
	snprintf(fs_path, sizeof(fs_path), LINUX_GPIO_LEVEL_FMT, gpio);
	if (NULL == (fp = fopen(fs_path, "rb+"))) {
		trace_error("err level fs_path=%s gpio not capable of level_state config\n", fs_path);
		return -1;
	}
	rewind(fp);

	fwrite(level_state[idx], sizeof(char), strlen(level_state[idx]), fp);
	fclose(fp);

	return ret;
}
