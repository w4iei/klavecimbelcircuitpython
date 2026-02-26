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

typedef struct {
    uint32_t spi0_xfers;
    uint32_t spi1_xfers;
    uint32_t spi0_fail;
    uint32_t spi1_fail;
    uint32_t drdy_count[PHOTON_SENSORSCAN_MAX_BANKS];
    uint32_t samples_written;
    uint16_t last_rx_len;
    uint8_t last_rx[16];
    uint32_t cs_set_calls;
    uint32_t cs_set_mismatch;
    uint8_t last_cs_bank;
    uint8_t last_cs_target;
    uint8_t last_cs_out;
    uint8_t last_cs_readback;
    uint8_t last_cs_direction;
    uint8_t cs_funcsel;
    uint8_t cs_pad_pue;
    uint8_t cs_pad_pde;
    uint8_t sck_funcsel[2];
    uint8_t mosi_funcsel[2];
    uint8_t miso_funcsel[2];
    uint32_t cs_funcsel_assert_fail;
    uint32_t spi_funcsel_assert_fail[2];
    uint32_t all_zero_rx_count[2];
    uint8_t miso_probe_pull_up[2];
    uint8_t miso_probe_pull_down[2];
    uint8_t miso_probe_valid_mask;
    uint8_t miso_probe_triggered_mask;
    uint8_t sanity_status_reg[PHOTON_SENSORSCAN_MAX_BANKS];
    uint8_t sanity_pin_cfg_reg[PHOTON_SENSORSCAN_MAX_BANKS];
    uint8_t sanity_general_cfg_reg[PHOTON_SENSORSCAN_MAX_BANKS];
    uint16_t sanity_status_valid_mask;
    uint16_t sanity_pin_cfg_valid_mask;
    uint16_t sanity_general_cfg_valid_mask;
    uint8_t last_bank;
    uint8_t last_bus;
    uint32_t last_error;
} photonsensorscan_debug_state_t;

// Python should pass `readings_buffer` as `array('H')` (recommended):
// each element is one unsigned 16-bit sensor reading. `bytearray` is also supported.
// SPI tuples may be omitted (passed as None) for buses not referenced by bank_spi_bus.
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
mp_obj_t common_hal_photonsensorscan_refresh_all_return(void);
bool common_hal_photonsensorscan_brownout(mp_int_t bank);
bool common_hal_photonsensorscan_crcerr_fuse(mp_int_t bank);
void common_hal_photonsensorscan_reset_device(mp_int_t bank);
void common_hal_photonsensorscan_reset_all_banks(void);
void common_hal_photonsensorscan_refresh_status(void);
uint8_t common_hal_photonsensorscan_slot_count(void);
void common_hal_photonsensorscan_get_debug_state(photonsensorscan_debug_state_t *out_state);
