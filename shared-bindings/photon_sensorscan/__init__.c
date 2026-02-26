// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/runtime.h"

#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/photon_sensorscan/__init__.h"

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
//|     bank_count: int,
//|     cs: tuple[microcontroller.Pin, ...],
//|     bank_spi_bus: tuple[int, ...],
//|     adc_channels: tuple[int, ...],
//|     emitter_bits: tuple[int, ...],
//|     readings_buffer: WriteableBuffer,
//|     settle_us: int = 18,
//|     baudrate: int = 15000000,
//|     polarity: int = 0,
//|     phase: int = 0,
//| ) -> None:
//|     """Initialize scanner hardware and bind a persistent output buffer for all bank/slot values."""
//|     ...
//|
static mp_obj_t photonsensorscan_init(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_spi0,
        ARG_spi1,
        ARG_bank_count,
        ARG_cs,
        ARG_bank_spi_bus,
        ARG_adc_channels,
        ARG_emitter_bits,
        ARG_readings_buffer,
        ARG_settle_us,
        ARG_baudrate,
        ARG_polarity,
        ARG_phase,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi0, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_spi1, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_bank_count, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
        { MP_QSTR_cs, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_bank_spi_bus, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_adc_channels, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_emitter_bits, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_readings_buffer, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
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
    uint8_t bank_count = (uint8_t)mp_arg_validate_int_range(
        args[ARG_bank_count].u_int, 1, PHOTON_SENSORSCAN_MAX_BANKS, MP_QSTR_bank_count);

    size_t cs_len = 0;
    mp_obj_t *cs_items = NULL;
    mp_obj_get_array(args[ARG_cs].u_obj, &cs_len, &cs_items);
    if (cs_len != bank_count) {
        mp_raise_ValueError(MP_ERROR_TEXT("cs len must match bank_count"));
    }
    const mcu_pin_obj_t *cs_pins[PHOTON_SENSORSCAN_MAX_BANKS];
    for (size_t i = 0; i < bank_count; i++) {
        cs_pins[i] = validate_obj_is_pin(cs_items[i], MP_QSTR_cs);
    }

    size_t bank_spi_bus_len = 0;
    mp_obj_t *bank_spi_bus_items = NULL;
    mp_obj_get_array(args[ARG_bank_spi_bus].u_obj, &bank_spi_bus_len, &bank_spi_bus_items);
    if (bank_spi_bus_len != bank_count) {
        mp_raise_ValueError(MP_ERROR_TEXT("bank_spi_bus len must match bank_count"));
    }
    uint8_t bank_spi_bus[PHOTON_SENSORSCAN_MAX_BANKS] = {0};
    for (size_t i = 0; i < bank_count; i++) {
        bank_spi_bus[i] = (uint8_t)mp_arg_validate_int_range(
            mp_obj_get_int(bank_spi_bus_items[i]), 0, 1, MP_QSTR_bank_spi_bus);
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
    if (adc_len > PHOTON_SENSORSCAN_MAX_SLOTS) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many slots"));
    }

    uint8_t adc_channels[PHOTON_SENSORSCAN_MAX_SLOTS] = {0};
    int8_t emitter_bits[PHOTON_SENSORSCAN_MAX_SLOTS] = {0};
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
        bank_count,
        cs_pins,
        bank_spi_bus,
        (uint8_t)adc_len, adc_channels, emitter_bits,
        args[ARG_readings_buffer].u_obj,
        settle_us, baudrate, polarity, phase);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(photonsensorscan_init_obj, 0, photonsensorscan_init);

//| def deinit() -> None:
//|     """Release module-owned SPI and CS resources so :func:`init` can fully reconstruct them."""
//|     ...
//|
static mp_obj_t photonsensorscan_deinit(void) {
    common_hal_photonsensorscan_deinit();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(photonsensorscan_deinit_obj, photonsensorscan_deinit);

//| def refresh_sensor(bank: int, slot: int) -> int:
//|     """Refresh one sensor value into the init-time buffer and return the new value."""
//|     ...
//|
static mp_obj_t photonsensorscan_refresh_sensor(mp_obj_t bank_in, mp_obj_t slot_in) {
    return mp_obj_new_int_from_uint(common_hal_photonsensorscan_refresh_sensor(mp_obj_get_int(bank_in), mp_obj_get_int(slot_in)));
}
MP_DEFINE_CONST_FUN_OBJ_2(photonsensorscan_refresh_sensor_obj, photonsensorscan_refresh_sensor);

//| def refresh_bank(bank: int) -> None:
//|     """Refresh every configured slot in one bank into the init-time buffer."""
//|     ...
//|
static mp_obj_t photonsensorscan_refresh_bank(mp_obj_t bank_in) {
    common_hal_photonsensorscan_refresh_bank(mp_obj_get_int(bank_in));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(photonsensorscan_refresh_bank_obj, photonsensorscan_refresh_bank);

//| def refresh_all() -> None:
//|     """Refresh all banks and slots into the init-time buffer."""
//|     ...
//|
static mp_obj_t photonsensorscan_refresh_all(void) {
    common_hal_photonsensorscan_refresh_all();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(photonsensorscan_refresh_all_obj, photonsensorscan_refresh_all);

//| def brownout(bank: int) -> bool:
//|     """Return the latched brownout status captured during init/reset for ``bank``."""
//|     ...
//|
static mp_obj_t photonsensorscan_brownout(mp_obj_t bank_in) {
    return mp_obj_new_bool(common_hal_photonsensorscan_brownout(mp_obj_get_int(bank_in)));
}
MP_DEFINE_CONST_FUN_OBJ_1(photonsensorscan_brownout_obj, photonsensorscan_brownout);

//| def crcerr_fuse(bank: int) -> bool:
//|     """Return the CRCERR_FUSE status captured during init/reset for ``bank``."""
//|     ...
//|
static mp_obj_t photonsensorscan_crcerr_fuse(mp_obj_t bank_in) {
    return mp_obj_new_bool(common_hal_photonsensorscan_crcerr_fuse(mp_obj_get_int(bank_in)));
}
MP_DEFINE_CONST_FUN_OBJ_1(photonsensorscan_crcerr_fuse_obj, photonsensorscan_crcerr_fuse);

//| def reset_device(bank: int) -> None:
//|     """Issue a device-level reset by setting ``GENERAL_CFG.RST`` for ``bank``."""
//|     ...
//|
static mp_obj_t photonsensorscan_reset_device(mp_obj_t bank_in) {
    common_hal_photonsensorscan_reset_device(mp_obj_get_int(bank_in));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(photonsensorscan_reset_device_obj, photonsensorscan_reset_device);

//| def reset_all_banks() -> None:
//|     """Reset and reconfigure all banks, then refresh cached status bits."""
//|     ...
//|
static mp_obj_t photonsensorscan_reset_all_banks(void) {
    common_hal_photonsensorscan_reset_all_banks();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(photonsensorscan_reset_all_banks_obj, photonsensorscan_reset_all_banks);

//| def refresh_status() -> None:
//|     """Read SYSTEM_STATUS for all banks and refresh cached status bits."""
//|     ...
//|
static mp_obj_t photonsensorscan_refresh_status(void) {
    common_hal_photonsensorscan_refresh_status();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(photonsensorscan_refresh_status_obj, photonsensorscan_refresh_status);

static const mp_rom_map_elem_t photonsensorscan_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_photon_sensorscan) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&photonsensorscan_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&photonsensorscan_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_sensor), MP_ROM_PTR(&photonsensorscan_refresh_sensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_bank), MP_ROM_PTR(&photonsensorscan_refresh_bank_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_all), MP_ROM_PTR(&photonsensorscan_refresh_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_brownout), MP_ROM_PTR(&photonsensorscan_brownout_obj) },
    { MP_ROM_QSTR(MP_QSTR_crcerr_fuse), MP_ROM_PTR(&photonsensorscan_crcerr_fuse_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_device), MP_ROM_PTR(&photonsensorscan_reset_device_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_all_banks), MP_ROM_PTR(&photonsensorscan_reset_all_banks_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_status), MP_ROM_PTR(&photonsensorscan_refresh_status_obj) },
};
static MP_DEFINE_CONST_DICT(photonsensorscan_module_globals, photonsensorscan_module_globals_table);

const mp_obj_module_t photonsensorscan_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&photonsensorscan_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_photon_sensorscan, photonsensorscan_module);
