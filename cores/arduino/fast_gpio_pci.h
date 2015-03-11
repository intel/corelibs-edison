/*
fast_gpio_pci.c function prototypes
Copyright (C) 2014 Intel Corporation

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

Implement a fast GPIO path by directly accessing to GPIO registers being
exposed to user-space as PCI resource files.

Author : Nicolas Pernas Maradei <nicolas.pernas.maradei@emutex.com> 2014
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
