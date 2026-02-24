// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#include "common-hal/microcontroller/Pin.h"
#include "py/obj.h"

void common_hal_photonsensorscan_init(
    const mcu_pin_obj_t *spi0_sck, const mcu_pin_obj_t *spi0_mosi, const mcu_pin_obj_t *spi0_miso,
    const mcu_pin_obj_t *spi1_sck, const mcu_pin_obj_t *spi1_mosi, const mcu_pin_obj_t *spi1_miso,
    const mcu_pin_obj_t *const cs_in[8],
    uint8_t slot_count_in, const uint8_t adc_channels_in[8], const int8_t emitter_bits_in[8],
    uint32_t settle_us_in, uint32_t baudrate, uint8_t polarity, uint8_t phase);

uint16_t common_hal_photonsensorscan_read_sensor(mp_int_t bank, mp_int_t slot);
void common_hal_photonsensorscan_scan_bank_into(mp_int_t bank, mp_obj_t out);
uint8_t common_hal_photonsensorscan_slot_count(void);
