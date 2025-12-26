// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/photon_sensorscan/Scanner.h"

#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/util.h"
#include "shared/runtime/context_manager_helpers.h"

#include "py/binary.h"
#include "py/objproperty.h"
#include "py/runtime.h"

//| class Scanner:
//|     """Fast, deterministic sensor scanning engine for photon boards."""
//|
//|     def __init__(
//|         self,
//|         enable_pins: tuple[microcontroller.Pin, ...] | list[microcontroller.Pin],
//|         sel0_pin: microcontroller.Pin,
//|         sel1_pin: microcontroller.Pin,
//|         adc_pin: microcontroller.Pin | int,
//|         *,
//|         settle_us: int,
//|         samples_per_channel: int = 1,
//|         sensors_per_bank: int,
//|         total_sensors: int,
//|     ) -> None:
//|         """Create a scanner bound to GPIO bank enables, mux selects, and one ADC input.
//|
//|         ``adc_pin`` can be an ADC-capable pin or an ADC channel index.
//|         When ``samples_per_channel`` is greater than 1, samples are averaged.
//|         """
//|         ...
//|
static mp_obj_t photon_sensorscan_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    photon_sensorscan_scanner_obj_t *self = mp_obj_malloc_with_finaliser(photon_sensorscan_scanner_obj_t, &photon_sensorscan_scanner_type);

    enum {
        ARG_enable_pins,
        ARG_sel0_pin,
        ARG_sel1_pin,
        ARG_adc_pin,
        ARG_settle_us,
        ARG_samples_per_channel,
        ARG_sensors_per_bank,
        ARG_total_sensors,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_enable_pins, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_sel0_pin, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_sel1_pin, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_adc_pin, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_settle_us, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
        { MP_QSTR_samples_per_channel, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1} },
        { MP_QSTR_sensors_per_bank, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
        { MP_QSTR_total_sensors, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_t enable_pins_obj = args[ARG_enable_pins].u_obj;
    validate_no_duplicate_pins(enable_pins_obj, MP_QSTR_enable_pins);

    size_t enable_pins_len = (size_t)MP_OBJ_SMALL_INT_VALUE(mp_obj_len(enable_pins_obj));
    if (enable_pins_len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("enable_pins empty"));
    }
    if (enable_pins_len > 0xFF) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many enable_pins"));
    }
    const mcu_pin_obj_t *enable_pins[enable_pins_len];
    for (size_t i = 0; i < enable_pins_len; i++) {
        enable_pins[i] = validate_obj_is_free_pin(
            mp_obj_subscr(enable_pins_obj, MP_OBJ_NEW_SMALL_INT(i), MP_OBJ_SENTINEL),
            MP_QSTR_enable_pins);
    }

    const mcu_pin_obj_t *sel0_pin = validate_obj_is_free_pin(args[ARG_sel0_pin].u_obj, MP_QSTR_sel0_pin);
    const mcu_pin_obj_t *sel1_pin = validate_obj_is_free_pin(args[ARG_sel1_pin].u_obj, MP_QSTR_sel1_pin);
    if (sel0_pin == sel1_pin) {
        mp_raise_ValueError(MP_ERROR_TEXT("pins must be distinct"));
    }
    for (size_t i = 0; i < enable_pins_len; i++) {
        if (enable_pins[i] == sel0_pin || enable_pins[i] == sel1_pin) {
            mp_raise_ValueError(MP_ERROR_TEXT("pins must be distinct"));
        }
    }

    mp_obj_t adc_obj = args[ARG_adc_pin].u_obj;
    const mcu_pin_obj_t *adc_pin = NULL;
    int adc_channel = -1;
    if (mp_obj_is_int(adc_obj)) {
        adc_channel = mp_obj_get_int(adc_obj);
    } else {
        adc_pin = validate_obj_is_free_pin(adc_obj, MP_QSTR_adc_pin);
    }
    if (adc_pin != NULL) {
        if (adc_pin == sel0_pin || adc_pin == sel1_pin) {
            mp_raise_ValueError(MP_ERROR_TEXT("pins must be distinct"));
        }
        for (size_t i = 0; i < enable_pins_len; i++) {
            if (enable_pins[i] == adc_pin) {
                mp_raise_ValueError(MP_ERROR_TEXT("pins must be distinct"));
            }
        }
    }

    uint16_t settle_us = (uint16_t)mp_arg_validate_int_range(args[ARG_settle_us].u_int, 0, 0xFFFF, MP_QSTR_settle_us);
    uint8_t samples_per_channel = (uint8_t)mp_arg_validate_int_range(args[ARG_samples_per_channel].u_int, 1, 0xFF, MP_QSTR_samples_per_channel);
    uint8_t sensors_per_bank = (uint8_t)mp_arg_validate_int_range(args[ARG_sensors_per_bank].u_int, 1, 0xFF, MP_QSTR_sensors_per_bank);
    uint16_t total_sensors = (uint16_t)mp_arg_validate_int_range(args[ARG_total_sensors].u_int, 1, 0xFFFF, MP_QSTR_total_sensors);

    common_hal_photon_sensorscan_construct(self, enable_pins, enable_pins_len,
        sel0_pin, sel1_pin, adc_pin, adc_channel,
        settle_us, samples_per_channel, sensors_per_bank, total_sensors);

    return MP_OBJ_FROM_PTR(self);
}

//|     def deinit(self) -> None:
//|         """Deinitialize the scanner and release its pins."""
//|         ...
//|
static mp_obj_t photon_sensorscan_obj_deinit(mp_obj_t self_in) {
    photon_sensorscan_scanner_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_photon_sensorscan_deinit(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(photon_sensorscan_deinit_obj, photon_sensorscan_obj_deinit);

static void check_for_deinit(photon_sensorscan_scanner_obj_t *self) {
    if (common_hal_photon_sensorscan_deinited(self)) {
        raise_deinited_error();
    }
}

//|     def scan_into(self, buffer: WriteableBuffer) -> None:
//|         """Scan the full sensor array into ``buffer`` (array('H') or bytearray)."""
//|         ...
//|
static mp_obj_t photon_sensorscan_obj_scan_into(mp_obj_t self_in, mp_obj_t buffer_in) {
    photon_sensorscan_scanner_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.typecode != 'H' && bufinfo.typecode != BYTEARRAY_TYPECODE) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("%q must be a bytearray or array of type 'H'"), MP_QSTR_buffer);
    }

    size_t required = (size_t)self->total_sensors * sizeof(uint16_t);
    if (bufinfo.len < required) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
    }

    common_hal_photon_sensorscan_scan_into(self, (uint16_t *)bufinfo.buf);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(photon_sensorscan_scan_into_obj, photon_sensorscan_obj_scan_into);

//|     def scan_indices_into(
//|         self,
//|         buffer: WriteableBuffer,
//|         indices: tuple[int, ...] | list[int],
//|     ) -> None:
//|         """Scan selected sensors into ``buffer`` (array('H') or bytearray).
//|
//|         ``indices`` uses the full scan order, and values are written in that order.
//|         """
//|         ...
//|
static mp_obj_t photon_sensorscan_obj_scan_indices_into(mp_obj_t self_in, mp_obj_t buffer_in, mp_obj_t indices_in) {
    photon_sensorscan_scanner_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.typecode != 'H' && bufinfo.typecode != BYTEARRAY_TYPECODE) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("%q must be a bytearray or array of type 'H'"), MP_QSTR_buffer);
    }

    size_t indices_len = 0;
    mp_obj_t *indices_items = NULL;
    mp_obj_get_array(indices_in, &indices_len, &indices_items);

    size_t required = indices_len * sizeof(uint16_t);
    if (bufinfo.len < required) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
    }

    uint16_t *buffer = (uint16_t *)bufinfo.buf;
    for (size_t i = 0; i < indices_len; i++) {
        mp_int_t index = mp_arg_validate_index_range(
            mp_obj_get_int(indices_items[i]),
            0,
            (mp_int_t)self->total_sensors - 1,
            MP_QSTR_index);
        uint16_t sensor_index = (uint16_t)index;
        uint8_t bank = sensor_index / self->sensors_per_bank;
        uint8_t channel = sensor_index % self->sensors_per_bank;
        buffer[i] = common_hal_photon_sensorscan_read_channel(self, bank, channel);
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(photon_sensorscan_scan_indices_into_obj, photon_sensorscan_obj_scan_indices_into);

//|     update_id: int
//|     """Monotonic counter incremented after each full scan. (read-only)"""
static mp_obj_t photon_sensorscan_get_update_id(mp_obj_t self_in) {
    photon_sensorscan_scanner_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);
    return mp_obj_new_int_from_uint(common_hal_photon_sensorscan_get_update_id(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(photon_sensorscan_get_update_id_obj, photon_sensorscan_get_update_id);

MP_PROPERTY_GETTER(photon_sensorscan_update_id_obj,
    (mp_obj_t)&photon_sensorscan_get_update_id_obj);

//|     def read_channel(self, bank: int, channel: int) -> int:
//|         """Read a single bank/channel (debug helper)."""
//|         ...
//|
static mp_obj_t photon_sensorscan_obj_read_channel(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_bank, ARG_channel };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bank, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_channel, MP_ARG_REQUIRED | MP_ARG_INT },
    };

    photon_sensorscan_scanner_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    check_for_deinit(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t bank = (uint8_t)mp_arg_validate_int_range(args[ARG_bank].u_int, 0, self->bank_count - 1, MP_QSTR_bank);
    uint8_t channel = (uint8_t)mp_arg_validate_int_range(args[ARG_channel].u_int, 0, self->sensors_per_bank - 1, MP_QSTR_channel);

    uint16_t value = common_hal_photon_sensorscan_read_channel(self, bank, channel);
    return MP_OBJ_NEW_SMALL_INT(value);
}
MP_DEFINE_CONST_FUN_OBJ_KW(photon_sensorscan_read_channel_obj, 1, photon_sensorscan_obj_read_channel);

static const mp_rom_map_elem_t photon_sensorscan_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&photon_sensorscan_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&default___exit___obj) },
    { MP_ROM_QSTR(MP_QSTR_scan_into), MP_ROM_PTR(&photon_sensorscan_scan_into_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan_indices_into), MP_ROM_PTR(&photon_sensorscan_scan_indices_into_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_channel), MP_ROM_PTR(&photon_sensorscan_read_channel_obj) },
    { MP_ROM_QSTR(MP_QSTR_update_id), MP_ROM_PTR(&photon_sensorscan_update_id_obj) },
};
static MP_DEFINE_CONST_DICT(photon_sensorscan_locals_dict, photon_sensorscan_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    photon_sensorscan_scanner_type,
    MP_QSTR_Scanner,
    MP_TYPE_FLAG_NONE,
    make_new, photon_sensorscan_make_new,
    locals_dict, &photon_sensorscan_locals_dict
    );
