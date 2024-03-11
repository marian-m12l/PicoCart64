/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#pragma once

#include <stdint.h>

#define EEPROM_16KBIT_SIZE        0x00000800
#define EEPROM_4KBIT_SIZE         0x00000200

extern uint8_t eeprom[EEPROM_4KBIT_SIZE];  //EEPROM_16KBIT_SIZE

void eeprom_load_from_flash(void);
void eeprom_save_to_flash(void);
