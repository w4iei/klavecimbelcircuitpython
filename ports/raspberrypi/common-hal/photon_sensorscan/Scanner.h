// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "common-hal/microcontroller/Pin.h"
#include "py/obj.h"

typedef struct {
    mp_obj_base_t base;
    uint8_t *enable_pin_numbers;
    uint8_t bank_count;
    uint8_t sel0_pin;
    uint8_t sel1_pin;
    uint8_t adc_pin_number;
    uint8_t adc_channel;
    uint16_t settle_us;
    uint8_t samples_per_channel;
    uint8_t sensors_per_bank;
    uint16_t total_sensors;
    uint32_t update_id;
} photon_sensorscan_scanner_obj_t;

void common_hal_photon_sensorscan_construct(photon_sensorscan_scanner_obj_t *self,
    const mcu_pin_obj_t **enable_pins, size_t enable_pins_len,
    const mcu_pin_obj_t *sel0_pin, const mcu_pin_obj_t *sel1_pin,
    const mcu_pin_obj_t *adc_pin, int adc_channel,
    uint16_t settle_us, uint8_t samples_per_channel,
    uint8_t sensors_per_bank, uint16_t total_sensors);
bool common_hal_photon_sensorscan_deinited(photon_sensorscan_scanner_obj_t *self);
void common_hal_photon_sensorscan_deinit(photon_sensorscan_scanner_obj_t *self);
void common_hal_photon_sensorscan_scan_into(photon_sensorscan_scanner_obj_t *self, uint16_t *buffer);
uint32_t common_hal_photon_sensorscan_get_update_id(photon_sensorscan_scanner_obj_t *self);
uint16_t common_hal_photon_sensorscan_read_channel(photon_sensorscan_scanner_obj_t *self,
    uint8_t bank_index, uint8_t channel);
