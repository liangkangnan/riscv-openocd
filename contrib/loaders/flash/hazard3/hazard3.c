// SPDX-License-Identifier: GPL-2.0-or-later

#include <stdint.h>

#define FLASH_BUSY	(1 << 1)

void flash_write(volatile uint32_t *flash_csr,
		uint32_t bytes_count,
		uint8_t *buffer,
		volatile uint32_t *target_addr) __attribute__((naked));

void flash_write(volatile uint32_t *flash_csr,
		uint32_t bytes_count,
		uint8_t *buffer,
		volatile uint32_t *target_addr)
{
	// enable direct mode
	*flash_csr |= 0x1;

	do {
		*target_addr = *buffer++;

		register uint32_t sr;
		do {
			sr = *flash_csr;
		} while (sr & FLASH_BUSY);

	} while (--bytes_count);

	// disable direct mode
	*flash_csr &= ~0x1;

	asm("ebreak");
}
