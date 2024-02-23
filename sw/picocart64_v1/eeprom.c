/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#include <string.h>
#include <stdint.h>

#include "eeprom.h"

#include "pico/stdlib.h"
#include "hardware/flash.h"

uint8_t eeprom[EEPROM_16KBIT_SIZE];

// eeprom_backup will be aligned to 4096 to match the flash sector erase size
uint16_t __aligned(4096) __attribute__((section(".n64_eeprom"))) eeprom_backup[EEPROM_16KBIT_SIZE];

void eeprom_load_from_flash(void)
{
	memcpy(eeprom, eeprom_backup, sizeof(eeprom));
}

void eeprom_save_to_flash(void)
{
	uint32_t offset = ((uint32_t) eeprom_backup) - XIP_BASE;
	uint32_t count = sizeof(eeprom);

	flash_range_erase(offset, count);
	flash_range_program(offset, (const uint8_t *)eeprom, count);
}
