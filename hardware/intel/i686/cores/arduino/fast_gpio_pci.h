/*
 * fast_gpio_pci.c
 *
 * Implement a fast GPIO path by directly accessing to GPIO registers being
 * exposed to user-space as PCI resource files.
 *
 * Author : Nicolas Pernas Maradei <nicolas.pernas.maradei@emutex.com> 2014
 */

#ifndef __FAST_GPIO_PCI_H__
#define __FAST_GPIO_PCI_H__

#ifdef __cplusplus
extern "C" {
#endif

int fastGpioPciInit(void);
void fastGpioPciFini(void);
void fastGpioPciDigitalWrite(register uint8_t id, register uint8_t val);
uint8_t fastGpioPciDigitalRead(register uint8_t id);
uint32_t fastGpioPciDigitalRegSnapshot(register uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __FAST_GPIO_SC_H__ */
