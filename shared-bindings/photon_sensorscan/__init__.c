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

static void parse_optional_spi_tuple(mp_obj_t tuple_obj, qstr arg_name,
    const mcu_pin_obj_t **sck, const mcu_pin_obj_t **mosi, const mcu_pin_obj_t **miso) {
    if (tuple_obj == mp_const_none) {
        *sck = NULL;
        *mosi = NULL;
        *miso = NULL;
        return;
    }
    parse_spi_tuple(tuple_obj, arg_name, sck, mosi, miso);
}

//| """Photon TLA2518 real-time scanner helpers."""
//|
//| def init(
//|     *,
//|     spi0: Optional[tuple[microcontroller.Pin, microcontroller.Pin, microcontroller.Pin]] = None,
//|     spi1: Optional[tuple[microcontroller.Pin, microcontroller.Pin, microcontroller.Pin]] = None,
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
//|     osr_mode: int = 0,
//| ) -> None:
//|     """Initialize scanner hardware and bind a persistent output buffer for all bank/slot values.
//|
//|     :param int osr_mode: Oversampling ratio (0=none, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x, 7=128x).
//|                          When OSR > 0, ADC outputs 16-bit values instead of 12-bit.
//|     """
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
        ARG_osr_mode,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi0, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_spi1, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
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
        { MP_QSTR_osr_mode, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const mcu_pin_obj_t *spi0_sck;
    const mcu_pin_obj_t *spi0_mosi;
    const mcu_pin_obj_t *spi0_miso;
    const mcu_pin_obj_t *spi1_sck;
    const mcu_pin_obj_t *spi1_mosi;
    const mcu_pin_obj_t *spi1_miso;
    parse_optional_spi_tuple(args[ARG_spi0].u_obj, MP_QSTR_spi0, &spi0_sck, &spi0_mosi, &spi0_miso);
    parse_optional_spi_tuple(args[ARG_spi1].u_obj, MP_QSTR_spi1, &spi1_sck, &spi1_mosi, &spi1_miso);
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
    uint8_t osr_mode = (uint8_t)mp_arg_validate_int_range(args[ARG_osr_mode].u_int, 0, 7, MP_QSTR_osr_mode);

    common_hal_photonsensorscan_init(
        spi0_sck, spi0_mosi, spi0_miso,
        spi1_sck, spi1_mosi, spi1_miso,
        bank_count,
        cs_pins,
        bank_spi_bus,
        (uint8_t)adc_len, adc_channels, emitter_bits,
        args[ARG_readings_buffer].u_obj,
        settle_us, baudrate, polarity, phase, osr_mode);
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

//| def refresh_all_return() -> tuple:
//|     """Temporary debug helper: read all banks/slots and return values as a flat tuple.
//|
//|     Values are ordered bank-major and this call does not write the init-time readings buffer.
//|     """
//|     ...
//|
static mp_obj_t photonsensorscan_refresh_all_return(void) {
    return common_hal_photonsensorscan_refresh_all_return();
}
MP_DEFINE_CONST_FUN_OBJ_0(photonsensorscan_refresh_all_return_obj, photonsensorscan_refresh_all_return);

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

//| def process_scan_events(
//|     *,
//|     active_sensors: int,
//|     latest_values: WriteableBuffer,
//|     sensor_disabled: WriteableBuffer,
//|     sensor_min: WriteableBuffer,
//|     sensor_max: WriteableBuffer,
//|     sensor_polarity: WriteableBuffer,
//|     sensor_strike_pct: WriteableBuffer,
//|     sensor_on: WriteableBuffer,
//|     sensor_active: WriteableBuffer,
//|     sensor_strike_pending: WriteableBuffer,
//|     sensor_strike_time_ms: WriteableBuffer,
//|     sensor_release_pending: WriteableBuffer,
//|     sensor_release_time_ms: WriteableBuffer,
//|     min_event_range: int,
//|     release_pct: int,
//|     activation_pct: int,
//|     velocity_window_pct: int = 20,
//|     strike_window_pct: int = 5,
//|     event_words: WriteableBuffer,
//| ) -> int:
//|     """Process one full scan in C and write compact events into ``event_words``.
//|
//|     ``event_words`` must be an ``array('H')`` where each event takes 3 words:
//|     ``(sensor_idx, state, dt_ms)``.
//|     """
//|     ...
//|
static mp_obj_t photonsensorscan_process_scan_events(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_active_sensors,
        ARG_latest_values,
        ARG_sensor_disabled,
        ARG_sensor_min,
        ARG_sensor_max,
        ARG_sensor_polarity,
        ARG_sensor_strike_pct,
        ARG_sensor_on,
        ARG_sensor_active,
        ARG_sensor_strike_pending,
        ARG_sensor_strike_time_ms,
        ARG_sensor_release_pending,
        ARG_sensor_release_time_ms,
        ARG_min_event_range,
        ARG_release_pct,
        ARG_activation_pct,
        ARG_velocity_window_pct,
        ARG_strike_window_pct,
        ARG_event_words,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_active_sensors, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
        { MP_QSTR_latest_values, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_disabled, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_min, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_max, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_polarity, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_strike_pct, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_on, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_active, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_strike_pending, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_strike_time_ms, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_release_pending, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_sensor_release_time_ms, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
        { MP_QSTR_min_event_range, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
        { MP_QSTR_release_pct, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
        { MP_QSTR_activation_pct, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
        { MP_QSTR_velocity_window_pct, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 20} },
        { MP_QSTR_strike_window_pct, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5} },
        { MP_QSTR_event_words, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    size_t event_count = common_hal_photonsensorscan_process_scan_events(
        args[ARG_active_sensors].u_int,
        args[ARG_latest_values].u_obj,
        args[ARG_sensor_disabled].u_obj,
        args[ARG_sensor_min].u_obj,
        args[ARG_sensor_max].u_obj,
        args[ARG_sensor_polarity].u_obj,
        args[ARG_sensor_strike_pct].u_obj,
        args[ARG_sensor_on].u_obj,
        args[ARG_sensor_active].u_obj,
        args[ARG_sensor_strike_pending].u_obj,
        args[ARG_sensor_strike_time_ms].u_obj,
        args[ARG_sensor_release_pending].u_obj,
        args[ARG_sensor_release_time_ms].u_obj,
        args[ARG_min_event_range].u_int,
        args[ARG_release_pct].u_int,
        args[ARG_activation_pct].u_int,
        args[ARG_velocity_window_pct].u_int,
        args[ARG_strike_window_pct].u_int,
        args[ARG_event_words].u_obj);
    return mp_obj_new_int_from_uint((mp_uint_t)event_count);
}
MP_DEFINE_CONST_FUN_OBJ_KW(photonsensorscan_process_scan_events_obj, 0, photonsensorscan_process_scan_events);

//| def debug_state() -> dict:
//|     """Return a snapshot of low-level driver debug counters/state."""
//|     ...
//|
static mp_obj_t photonsensorscan_debug_state(void) {
    photonsensorscan_debug_state_t state;
    common_hal_photonsensorscan_get_debug_state(&state);

    mp_obj_t drdy_items[PHOTON_SENSORSCAN_MAX_BANKS];
    for (size_t i = 0; i < PHOTON_SENSORSCAN_MAX_BANKS; i++) {
        drdy_items[i] = mp_obj_new_int_from_uint(state.drdy_count[i]);
    }
    mp_obj_t all_zero_items[2] = {
        mp_obj_new_int_from_uint(state.all_zero_rx_count[0]),
        mp_obj_new_int_from_uint(state.all_zero_rx_count[1]),
    };
    mp_obj_t spi_funcsel_assert_items[2] = {
        mp_obj_new_int_from_uint(state.spi_funcsel_assert_fail[0]),
        mp_obj_new_int_from_uint(state.spi_funcsel_assert_fail[1]),
    };

    mp_obj_t out = mp_obj_new_dict(39);
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_spi0_xfers), mp_obj_new_int_from_uint(state.spi0_xfers));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_spi1_xfers), mp_obj_new_int_from_uint(state.spi1_xfers));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_spi0_fail), mp_obj_new_int_from_uint(state.spi0_fail));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_spi1_fail), mp_obj_new_int_from_uint(state.spi1_fail));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_drdy_count), mp_obj_new_tuple(PHOTON_SENSORSCAN_MAX_BANKS, drdy_items));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_samples_written), mp_obj_new_int_from_uint(state.samples_written));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_rx_len), mp_obj_new_int_from_uint(state.last_rx_len));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_rx), mp_obj_new_bytes(state.last_rx, sizeof(state.last_rx)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_cs_set_calls), mp_obj_new_int_from_uint(state.cs_set_calls));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_cs_set_mismatch), mp_obj_new_int_from_uint(state.cs_set_mismatch));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_cs_bank), mp_obj_new_int_from_uint(state.last_cs_bank));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_cs_target), mp_obj_new_int_from_uint(state.last_cs_target));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_cs_out), mp_obj_new_int_from_uint(state.last_cs_out));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_cs_readback), mp_obj_new_int_from_uint(state.last_cs_readback));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_cs_direction), mp_obj_new_int_from_uint(state.last_cs_direction));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_cs_funcsel), mp_obj_new_int_from_uint(state.cs_funcsel));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_expected_spi_funcsel), mp_obj_new_int_from_uint(state.expected_spi_funcsel));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_expected_sio_funcsel), mp_obj_new_int_from_uint(state.expected_sio_funcsel));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_cs_pad_pue), mp_obj_new_int_from_uint(state.cs_pad_pue));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_cs_pad_pde), mp_obj_new_int_from_uint(state.cs_pad_pde));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_sck_funcsel), mp_obj_new_bytes(state.sck_funcsel, sizeof(state.sck_funcsel)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_mosi_funcsel), mp_obj_new_bytes(state.mosi_funcsel, sizeof(state.mosi_funcsel)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_miso_funcsel), mp_obj_new_bytes(state.miso_funcsel, sizeof(state.miso_funcsel)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_cs_funcsel_assert_fail), mp_obj_new_int_from_uint(state.cs_funcsel_assert_fail));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_spi_funcsel_assert_fail), mp_obj_new_tuple(2, spi_funcsel_assert_items));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_all_zero_rx_count), mp_obj_new_tuple(2, all_zero_items));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_miso_probe_pull_up), mp_obj_new_bytes(state.miso_probe_pull_up, sizeof(state.miso_probe_pull_up)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_miso_probe_pull_down), mp_obj_new_bytes(state.miso_probe_pull_down, sizeof(state.miso_probe_pull_down)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_miso_probe_valid_mask), mp_obj_new_int_from_uint(state.miso_probe_valid_mask));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_miso_probe_triggered_mask), mp_obj_new_int_from_uint(state.miso_probe_triggered_mask));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_sanity_status_reg), mp_obj_new_bytes(state.sanity_status_reg, sizeof(state.sanity_status_reg)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_sanity_pin_cfg_reg), mp_obj_new_bytes(state.sanity_pin_cfg_reg, sizeof(state.sanity_pin_cfg_reg)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_sanity_general_cfg_reg), mp_obj_new_bytes(state.sanity_general_cfg_reg, sizeof(state.sanity_general_cfg_reg)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_sanity_status_valid_mask), mp_obj_new_int_from_uint(state.sanity_status_valid_mask));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_sanity_pin_cfg_valid_mask), mp_obj_new_int_from_uint(state.sanity_pin_cfg_valid_mask));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_sanity_general_cfg_valid_mask), mp_obj_new_int_from_uint(state.sanity_general_cfg_valid_mask));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_bank), mp_obj_new_int_from_uint(state.last_bank));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_bus), mp_obj_new_int_from_uint(state.last_bus));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_last_error), mp_obj_new_int_from_uint(state.last_error));
    return out;
}
MP_DEFINE_CONST_FUN_OBJ_0(photonsensorscan_debug_state_obj, photonsensorscan_debug_state);

static mp_obj_t classify_miso_probe(uint8_t valid_mask, uint8_t bus, uint8_t up, uint8_t down) {
    if ((valid_mask & (1u << bus)) == 0) {
        return MP_OBJ_NEW_QSTR(MP_QSTR_not_probed);
    }
    if (up == 0 && down == 0) {
        return MP_OBJ_NEW_QSTR(MP_QSTR_stuck_low);
    }
    if (up == 1 && down == 0) {
        return MP_OBJ_NEW_QSTR(MP_QSTR_floating);
    }
    if (up == 1 && down == 1) {
        return MP_OBJ_NEW_QSTR(MP_QSTR_stuck_high);
    }
    return MP_OBJ_NEW_QSTR(MP_QSTR_invalid);
}

//| def debug_miso_probe() -> dict:
//|     """Return one-time MISO pull probe results captured after all-zero RX detection."""
//|     ...
//|
static mp_obj_t photonsensorscan_debug_miso_probe(void) {
    photonsensorscan_debug_state_t state;
    common_hal_photonsensorscan_get_debug_state(&state);
    mp_obj_t out = mp_obj_new_dict(7);
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_valid_mask), mp_obj_new_int_from_uint(state.miso_probe_valid_mask));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_triggered_mask), mp_obj_new_int_from_uint(state.miso_probe_triggered_mask));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_pull_up), mp_obj_new_bytes(state.miso_probe_pull_up, sizeof(state.miso_probe_pull_up)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_pull_down), mp_obj_new_bytes(state.miso_probe_pull_down, sizeof(state.miso_probe_pull_down)));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_bus0_state), classify_miso_probe(
        state.miso_probe_valid_mask, 0, state.miso_probe_pull_up[0], state.miso_probe_pull_down[0]));
    mp_obj_dict_store(out, MP_OBJ_NEW_QSTR(MP_QSTR_bus1_state), classify_miso_probe(
        state.miso_probe_valid_mask, 1, state.miso_probe_pull_up[1], state.miso_probe_pull_down[1]));
    return out;
}
MP_DEFINE_CONST_FUN_OBJ_0(photonsensorscan_debug_miso_probe_obj, photonsensorscan_debug_miso_probe);

static const mp_rom_map_elem_t photonsensorscan_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_photon_sensorscan) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&photonsensorscan_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&photonsensorscan_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_sensor), MP_ROM_PTR(&photonsensorscan_refresh_sensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_bank), MP_ROM_PTR(&photonsensorscan_refresh_bank_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_all), MP_ROM_PTR(&photonsensorscan_refresh_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_all_return), MP_ROM_PTR(&photonsensorscan_refresh_all_return_obj) },
    { MP_ROM_QSTR(MP_QSTR_brownout), MP_ROM_PTR(&photonsensorscan_brownout_obj) },
    { MP_ROM_QSTR(MP_QSTR_crcerr_fuse), MP_ROM_PTR(&photonsensorscan_crcerr_fuse_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_device), MP_ROM_PTR(&photonsensorscan_reset_device_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_all_banks), MP_ROM_PTR(&photonsensorscan_reset_all_banks_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_status), MP_ROM_PTR(&photonsensorscan_refresh_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_process_scan_events), MP_ROM_PTR(&photonsensorscan_process_scan_events_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug_state), MP_ROM_PTR(&photonsensorscan_debug_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug_miso_probe), MP_ROM_PTR(&photonsensorscan_debug_miso_probe_obj) },
};
static MP_DEFINE_CONST_DICT(photonsensorscan_module_globals, photonsensorscan_module_globals_table);

const mp_obj_module_t photonsensorscan_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&photonsensorscan_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_photon_sensorscan, photonsensorscan_module);
