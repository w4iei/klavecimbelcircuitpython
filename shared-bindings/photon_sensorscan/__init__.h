// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common-hal/microcontroller/Pin.h"
#include "py/obj.h"

#define PHOTON_SENSORSCAN_MAX_BANKS (16)
#define PHOTON_SENSORSCAN_MAX_SLOTS (8)

// Python should pass `readings_buffer` as `array('H')` (recommended):
// each element is one unsigned 16-bit sensor reading. `bytearray` is also supported.
void common_hal_photonsensorscan_init(
    const mcu_pin_obj_t *spi0_sck, const mcu_pin_obj_t *spi0_mosi, const mcu_pin_obj_t *spi0_miso,
    const mcu_pin_obj_t *spi1_sck, const mcu_pin_obj_t *spi1_mosi, const mcu_pin_obj_t *spi1_miso,
    uint8_t bank_count_in,
    const mcu_pin_obj_t *const cs_in[PHOTON_SENSORSCAN_MAX_BANKS],
    const uint8_t bank_spi_bus_in[PHOTON_SENSORSCAN_MAX_BANKS],
    uint8_t slot_count_in, const uint8_t adc_channels_in[PHOTON_SENSORSCAN_MAX_SLOTS], const int8_t emitter_bits_in[PHOTON_SENSORSCAN_MAX_SLOTS],
    mp_obj_t readings_buffer,
    uint32_t settle_us_in, uint32_t baudrate, uint8_t polarity, uint8_t phase);
void common_hal_photonsensorscan_deinit(void);

uint16_t common_hal_photonsensorscan_refresh_sensor(mp_int_t bank, mp_int_t slot);
void common_hal_photonsensorscan_refresh_bank(mp_int_t bank);
void common_hal_photonsensorscan_refresh_all(void);
bool common_hal_photonsensorscan_brownout(mp_int_t bank);
bool common_hal_photonsensorscan_crcerr_fuse(mp_int_t bank);
void common_hal_photonsensorscan_reset_device(mp_int_t bank);
void common_hal_photonsensorscan_reset_all_banks(void);
void common_hal_photonsensorscan_refresh_status(void);
uint8_t common_hal_photonsensorscan_slot_count(void);
