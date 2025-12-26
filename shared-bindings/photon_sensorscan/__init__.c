// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/runtime.h"

#include "shared-bindings/photon_sensorscan/Scanner.h"

//| """Photon-specific real-time sensor scanning helpers.
//|
//| This module provides a fast, C-backed scanner for banked sensor arrays.
//| Enable it per-board by setting ``CIRCUITPY_PHOTON_SENSORSCAN = 1`` in that
//| board's ``mpconfigboard.mk``.
//| """
//|
static const mp_rom_map_elem_t photon_sensorscan_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_photon_sensorscan) },
    { MP_ROM_QSTR(MP_QSTR_Scanner), MP_ROM_PTR(&photon_sensorscan_scanner_type) },
};
static MP_DEFINE_CONST_DICT(photon_sensorscan_module_globals, photon_sensorscan_module_globals_table);

const mp_obj_module_t photon_sensorscan_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&photon_sensorscan_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_photon_sensorscan, photon_sensorscan_module);
