// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/runtime.h"

#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/photonsensorscan/__init__.h"

static void parse_spi_tuple(mp_obj_t tuple_obj, qstr arg_name,
    const mcu_pin_obj_t **sck, const mcu_pin_obj_t **mosi, const mcu_pin_obj_t **miso) {
    size_t len = 0;
    mp_obj_t *items = NULL;
    mp_obj_get_array(tuple_obj, &len, &items);
    if (len != 3) {
        mp_raise_ValueError(MP_ERROR_TEXT("SPI tuple len must be 3"));
    }
    *sck = validate_obj_is_pin(items[0], arg_name);
    *mosi = validate_obj_is_pin(items[1], arg_name);
    *miso = validate_obj_is_pin(items[2], arg_name);
}

//| """Photon TLA2518 real-time scanner helpers."""
//|
//| def init(
//|     *,
//|     spi0: tuple[microcontroller.Pin, microcontroller.Pin, microcontroller.Pin],
//|     spi1: tuple[microcontroller.Pin, microcontroller.Pin, microcontroller.Pin],
//|     cs: tuple[microcontroller.Pin, microcontroller.Pin, microcontroller.Pin, microcontroller.Pin, microcontroller.Pin, microcontroller.Pin, microcontroller.Pin, microcontroller.Pin],
//|     adc_channels: tuple[int, ...],
//|     emitter_bits: tuple[int, ...],
//|     settle_us: int = 18,
//|     baudrate: int = 15000000,
//|     polarity: int = 0,
//|     phase: int = 0,
//| ) -> None:
//|     """Initialize module-owned SPI0/SPI1, CS lines, and per-slot channel/emitter config."""
//|     ...
//|
static mp_obj_t photonsensorscan_init(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_spi0,
        ARG_spi1,
        ARG_cs,
        ARG_adc_channels,
        ARG_emitter_bits,
        ARG_settle_us,
        ARG_baudrate,
        ARG_polarity,
        ARG_phase,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi0, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_spi1, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_cs, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_adc_channels, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_emitter_bits, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_settle_us, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 18} },
        { MP_QSTR_baudrate, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 15000000} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_phase, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const mcu_pin_obj_t *spi0_sck;
    const mcu_pin_obj_t *spi0_mosi;
    const mcu_pin_obj_t *spi0_miso;
    const mcu_pin_obj_t *spi1_sck;
    const mcu_pin_obj_t *spi1_mosi;
    const mcu_pin_obj_t *spi1_miso;
    parse_spi_tuple(args[ARG_spi0].u_obj, MP_QSTR_spi0, &spi0_sck, &spi0_mosi, &spi0_miso);
    parse_spi_tuple(args[ARG_spi1].u_obj, MP_QSTR_spi1, &spi1_sck, &spi1_mosi, &spi1_miso);

    size_t cs_len = 0;
    mp_obj_t *cs_items = NULL;
    mp_obj_get_array(args[ARG_cs].u_obj, &cs_len, &cs_items);
    if (cs_len != 8) {
        mp_raise_ValueError(MP_ERROR_TEXT("cs len must be 8"));
    }
    const mcu_pin_obj_t *cs_pins[8];
    for (size_t i = 0; i < 8; i++) {
        cs_pins[i] = validate_obj_is_pin(cs_items[i], MP_QSTR_cs);
    }

    size_t adc_len = 0;
    mp_obj_t *adc_items = NULL;
    mp_obj_get_array(args[ARG_adc_channels].u_obj, &adc_len, &adc_items);
    size_t emitter_len = 0;
    mp_obj_t *emitter_items = NULL;
    mp_obj_get_array(args[ARG_emitter_bits].u_obj, &emitter_len, &emitter_items);

    if (adc_len != emitter_len) {
        mp_raise_ValueError(MP_ERROR_TEXT("len mismatch"));
    }
    if (adc_len > 8) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many slots"));
    }

    uint8_t adc_channels[8] = {0};
    int8_t emitter_bits[8] = {0};
    for (size_t i = 0; i < adc_len; i++) {
        adc_channels[i] = (uint8_t)mp_arg_validate_int_range(mp_obj_get_int(adc_items[i]), 0, 7, MP_QSTR_adc_channels);
        emitter_bits[i] = (int8_t)mp_arg_validate_int_range(mp_obj_get_int(emitter_items[i]), -1, 7, MP_QSTR_emitter_bits);
    }

    uint32_t settle_us = (uint32_t)mp_arg_validate_int_min(args[ARG_settle_us].u_int, 0, MP_QSTR_settle_us);
    uint32_t baudrate = (uint32_t)mp_arg_validate_int_min(args[ARG_baudrate].u_int, 1, MP_QSTR_baudrate);
    uint8_t polarity = (uint8_t)mp_arg_validate_int_range(args[ARG_polarity].u_int, 0, 1, MP_QSTR_polarity);
    uint8_t phase = (uint8_t)mp_arg_validate_int_range(args[ARG_phase].u_int, 0, 1, MP_QSTR_phase);

    common_hal_photonsensorscan_init(
        spi0_sck, spi0_mosi, spi0_miso,
        spi1_sck, spi1_mosi, spi1_miso,
        cs_pins,
        (uint8_t)adc_len, adc_channels, emitter_bits,
        settle_us, baudrate, polarity, phase);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(photonsensorscan_init_obj, 0, photonsensorscan_init);

//| def read_sensor(bank: int, slot: int) -> int:
//|     """Read one slot from one bank."""
//|     ...
//|
static mp_obj_t photonsensorscan_read_sensor(mp_obj_t bank_in, mp_obj_t slot_in) {
    return mp_obj_new_int_from_uint(common_hal_photonsensorscan_read_sensor(mp_obj_get_int(bank_in), mp_obj_get_int(slot_in)));
}
MP_DEFINE_CONST_FUN_OBJ_2(photonsensorscan_read_sensor_obj, photonsensorscan_read_sensor);

//| def scan_bank_into(bank: int, out: WriteableBuffer) -> None:
//|     """Scan all configured slots for one bank into ``out``."""
//|     ...
//|
static mp_obj_t photonsensorscan_scan_bank_into(mp_obj_t bank_in, mp_obj_t out_in) {
    common_hal_photonsensorscan_scan_bank_into(mp_obj_get_int(bank_in), out_in);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(photonsensorscan_scan_bank_into_obj, photonsensorscan_scan_bank_into);

static const mp_rom_map_elem_t photonsensorscan_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_photon_sensorscan) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&photonsensorscan_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_sensor), MP_ROM_PTR(&photonsensorscan_read_sensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan_bank_into), MP_ROM_PTR(&photonsensorscan_scan_bank_into_obj) },
};
static MP_DEFINE_CONST_DICT(photonsensorscan_module_globals, photonsensorscan_module_globals_table);

const mp_obj_module_t photonsensorscan_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&photonsensorscan_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_photon_sensorscan, photonsensorscan_module);
MP_REGISTER_MODULE(MP_QSTR_photonsensorscan, photonsensorscan_module);
