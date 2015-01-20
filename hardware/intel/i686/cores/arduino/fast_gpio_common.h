#ifndef __FAST_GPIO_COMMON_H__
#define __FAST_GPIO_COMMON_H__

// Macros to (de-)construct GPIO_FAST_* register descriptors
#define GPIO_FAST_TYPE_NONE			0x00
#define GPIO_FAST_TYPE_QUARK_SC			0x01
#define GPIO_FAST_TYPE_QUARK_NC			0x02
#define GPIO_FAST_ID(type, rd_reg, wr_reg, mask) \
	(0UL | ((type) << 24) | ((rd_reg) << 16) | ((wr_reg) << 8) | (mask))
#define GPIO_FAST_ID_TYPE(id)	(((id) >> 24) & 0xFF)
#define GPIO_FAST_ID_RD_REG(id)	(((id) >> 16) & 0xFF)
#define GPIO_FAST_ID_WR_REG(id)	(((id) >> 8) & 0xFF)
#define GPIO_FAST_ID_MASK(id)	((id) & 0xFF)

int fastGpioFindUioByName(const char *name);

int fastGpioGetInfo(const int uio_num,
		    const int index,
		    char *path_fmt);

#endif // __FAST_GPIO_COMMON_H__
