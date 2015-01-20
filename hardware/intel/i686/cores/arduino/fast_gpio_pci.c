/*
 * fast_gpio_pci.c
 *
 * Implement a fast GPIO path by directly accessing to GPIO registers being
 * exposed to user-space as PCI resource files.
 *
 * Author : Nicolas Pernas Maradei <nicolas.pernas.maradei@emutex.com> 2014
 */

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <dirent.h>
#include <trace.h>
#include <Arduino.h>

#include "fast_gpio_pci.h"

#define MY_TRACE_PREFIX		"fast_gpio_pci"
#define PCI_BUS_DEVICES		"/sys/bus/pci/devices"
#define PCI_DEVICE_ATTR		(PCI_BUS_DEVICES "/%s/%s")
#define PCI_DEVICE_RES0		(PCI_BUS_DEVICES "/%s/resource0")
#define PCI_DEVICE_VENDOR	"vendor"
#define PCI_DEVICE_DEVICE	"device"
#define PCI_INTEL_VENDORID	"0x8086"
#define PCI_INTEL_DEVICEID	"0x1199"
#define GPIO_REG_SIZE		32
#define GPIO_REG_OFFSET(id)	(((id) / GPIO_REG_SIZE) * sizeof(uint32_t))
#define GPIO_GPLR		0x04
#define GPIO_GPSR		0x34
#define GPIO_GPCR		0x4c

struct fgpio_pci {
	uint8_t *regs;
	int res0_handle;
	int size;
};

struct fgpio_pci_info {
	uint8_t gpio;
	uint8_t offset;
};

static struct fgpio_pci_info fgpio_info[] = {
	{ 130, GPIO_REG_OFFSET(130) }, //IO0
	{ 131, GPIO_REG_OFFSET(131) }, //IO1
	{ 128, GPIO_REG_OFFSET(128) }, //IO2
	{  12, GPIO_REG_OFFSET(12) },  //IO3
	{ 129, GPIO_REG_OFFSET(129) }, //IO4
	{  13, GPIO_REG_OFFSET(13) },  //IO5
	{ 182, GPIO_REG_OFFSET(182) }, //IO6
	{  48, GPIO_REG_OFFSET(48) },  //IO7
	{  49, GPIO_REG_OFFSET(49) },  //IO8
	{ 183, GPIO_REG_OFFSET(183) }, //IO9
	{  41, GPIO_REG_OFFSET(41) },  //IO10
	{  43, GPIO_REG_OFFSET(43) },  //IO11
	{  42, GPIO_REG_OFFSET(42) },  //IO12
	{  40, GPIO_REG_OFFSET(40) },  //IO13
	{  44, GPIO_REG_OFFSET(44) },  //IO14
	{  45, GPIO_REG_OFFSET(45) },  //IO15
	{  46, GPIO_REG_OFFSET(46) },  //IO16
	{  47, GPIO_REG_OFFSET(47) },  //IO17
	{  14, GPIO_REG_OFFSET(14) },  //IO18
	{ 165, GPIO_REG_OFFSET(165) }  //IO19
};

static struct fgpio_pci fgpio;

static int
fastGpioPciReadAttr(const char *pci_bus_id, const char *attr, char *buf, int buflen)
{
	char path[PATH_MAX];
	int fd;
	int ret = 0;

	snprintf(path, sizeof(path), PCI_DEVICE_ATTR, pci_bus_id, attr);

	if ((fd = open(path, O_RDONLY)) < 0)
		return -1;

	if (read(fd, buf, buflen) < 0) {
		trace_error("%s Can't read %s: %s", __func__, path,
				strerror(errno));
		ret = -1;
	}
	close(fd);
	return ret;
}

static int
fastGpioPciFindResourceById(const char *vendorid, const char *deviceid,
		char *path, int pathlen)
{
	DIR *dir;
	const struct dirent *ent;
	char buf[PATH_MAX];

	if ((dir = opendir(PCI_BUS_DEVICES)) == NULL) {
		trace_error("%s Can't open PCI devices directory: %s", __func__,
				strerror(errno));
		return -1;
	}

	while ((ent = readdir(dir)) != NULL) {
		/* Get vendor id */
		if (fastGpioPciReadAttr(ent->d_name, PCI_DEVICE_VENDOR, buf,
				sizeof(buf)) < 0)
			continue;

		/* Skip the rest if vendor id does not match */
		if (strncmp(buf, vendorid, strlen(vendorid)))
			continue;

		/* Vendor id did match. Get device id */
		if (fastGpioPciReadAttr(ent->d_name, PCI_DEVICE_DEVICE, buf,
				sizeof(buf)) < 0)
			continue;

		/* If device id also matches, we found it! stop searching */
		if (!strncmp(buf, deviceid, strlen(deviceid)))
			break;
	}
	closedir(dir);

	if (ent == NULL) {
		trace_error("%s Can't find PCI device with vendor:%s device:%s",
				__func__, vendorid, deviceid);
		return -1;
	}

	snprintf(path, pathlen, PCI_DEVICE_RES0, ent->d_name);

	return 0;
}

/**
 * fastGpioPciInit
 *
 * Initialize the fast GPIO interface
 */
int fastGpioPciInit(void)
{
	char resource0[PATH_MAX];
	struct stat st;
	int fd;
	uint8_t *regs;

	fgpio.regs = NULL;
	fgpio.res0_handle = -1;

	if ((fastGpioPciFindResourceById(PCI_INTEL_VENDORID, PCI_INTEL_DEVICEID,
			resource0, sizeof(resource0))) < 0)
		return -1;

	if ((fd = open(resource0, O_RDWR)) < 0){
		trace_error("%s Can't open %s: %s", __func__, resource0, strerror(errno));
		return -1;
	}

	if (fstat(fd, &st) < 0) {
		trace_error("%s Can't get file size: %s", strerror(errno));
		return -1;
	}

	regs = (uint8_t*) mmap(NULL, st.st_size,
			PROT_READ | PROT_WRITE,
			MAP_FILE | MAP_SHARED, fd, 0);

	if (regs == MAP_FAILED) {
		trace_error("%s Unable to mmap %s: %s", __func__, resource0,
				strerror(errno));
		close(fd);
		return -1;
	}

	fgpio.res0_handle = fd;
	fgpio.size = st.st_size;
	fgpio.regs = regs;

	trace_debug("%s Mapped resource0:%s size:%d handle:%d", __func__,
		    resource0, fgpio.size, fgpio.res0_handle);

	return 0;
}

/**
 * fastGpioPciFini
 *
 * Tear down the fast SC GPIO interface
 */
void fastGpioPciFini(void)
{
	if (fgpio.regs != NULL) {
		munmap(fgpio.regs, fgpio.size);
		fgpio.regs = NULL;
	}

	if (fgpio.res0_handle != -1) {
		close(fgpio.res0_handle);
		fgpio.res0_handle = -1;
	}
}


/**
 * fastGpioPciDigitalWrite
 *
 * Do a fast write to the GPIO regs
 *
 */
void fastGpioPciDigitalWrite(uint8_t id, uint8_t val)
{
	struct fgpio_pci_info info = fgpio_info[id];
	uint8_t gpio_reg_offset;

	gpio_reg_offset = val ? GPIO_GPSR : GPIO_GPCR;

	/* GPIOs are enabled by writing 1 into the GPIO's bit in the register.
	 * Writing a 0 has no effect on the pins, so instead of doing a
	 * read/modify/write we just overwrite the whole register because we
	 * know there's only one bit set to 1. */
	*(volatile uint32_t*)
		(fgpio.regs + gpio_reg_offset + info.offset) =
				(uint32_t)(1 << (info.gpio % GPIO_REG_SIZE));
}

/**
 * fastGpioPciDigitalRead
 *
 * Do a fast read from the GPIO regs
 *
 */
uint8_t fastGpioPciDigitalRead(uint8_t id)
{
	struct fgpio_pci_info info = fgpio_info[id];
	uint32_t regval;

	regval = *(volatile uint32_t*) (fgpio.regs + GPIO_GPLR + info.offset);

	return regval & (uint32_t)(1 << (info.gpio % GPIO_REG_SIZE)) ? 1 : 0;
}

/**
 * fastGpioPciDigitalRegSnapshot
 *
 * Read the current state of the GPIO registers.
 *
 */
uint32_t fastGpioPciDigitalRegSnapshot(uint8_t id)
{
	/* The register varies depending on the action: read, set, clear.
	 * Returns nothing. */
	return 0;
}
