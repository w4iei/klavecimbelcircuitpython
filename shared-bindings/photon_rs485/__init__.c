// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/objstr.h"
#include "py/runtime.h"

#include "shared-bindings/photon_rs485/RS485.h"

static const uint8_t photon_rs485_frame_preamble[] = { 0xA5, 0x5A, 0xC3, 0x3C };
MP_DEFINE_BYTES_OBJ(photon_rs485_frame_preamble_obj, photon_rs485_frame_preamble, sizeof(photon_rs485_frame_preamble));

//| """Photon-specific RS485 framing helpers.
//|
//| This module provides a fast, C-backed RS485 framing driver with CRC32 for
//| photon RP2350A boards. It can auto-reply to multiple configured request types
//| when :py:meth:`photon_rs485.RS485.read_frames` is called.
//|
//| Enable it per-board by setting ``CIRCUITPY_PHOTON_RS485 = 1`` in that
//| board's ``mpconfigboard.mk``.
//|
//| Example (manual framing in Python)::
//|
//|     for frame_type, target_id, payload, seq in bus.read_frames():
//|         if frame_type == photon_rs485.FRAME_TYPE_PING:
//|             bus.send_frame(photon_rs485.FRAME_TYPE_PONG, device_id, b"", seq)
//| """
//|
static const mp_rom_map_elem_t photon_rs485_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_photon_rs485) },
    { MP_ROM_QSTR(MP_QSTR_RS485), MP_ROM_PTR(&photon_rs485_rs485_type) },

    { MP_ROM_QSTR(MP_QSTR_FRAME_PREAMBLE), MP_ROM_PTR(&photon_rs485_frame_preamble_obj) },
    { MP_ROM_QSTR(MP_QSTR_FRAME_HEADER_LEN), MP_ROM_INT(10) },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TRAILER_LEN), MP_ROM_INT(4) },

    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_PING), MP_ROM_INT('P') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_PONG), MP_ROM_INT('O') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_DATA_REQ), MP_ROM_INT('R') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_DATA_RESP), MP_ROM_INT('D') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_STATS_REQ), MP_ROM_INT('S') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_STATS_RESP), MP_ROM_INT('s') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_EVENT), MP_ROM_INT('E') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_EVENT_ACK), MP_ROM_INT('A') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_CFG_SET), MP_ROM_INT('C') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_CFG_ACK), MP_ROM_INT('c') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_MINMAX_REQ), MP_ROM_INT('M') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_MINMAX_RESP), MP_ROM_INT('m') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_TRACE_REQ), MP_ROM_INT('T') },
    { MP_ROM_QSTR(MP_QSTR_FRAME_TYPE_TRACE_RESP), MP_ROM_INT('t') },

    { MP_ROM_QSTR(MP_QSTR_DEVICE_MAIN), MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_DEVICE_SECONDARY_1), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_DEVICE_SECONDARY_2), MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_DEVICE_SECONDARY_3), MP_ROM_INT(3) },
    { MP_ROM_QSTR(MP_QSTR_DEVICE_SECONDARY_4), MP_ROM_INT(4) },
    { MP_ROM_QSTR(MP_QSTR_DEVICE_SECONDARY_5), MP_ROM_INT(5) },
};
static MP_DEFINE_CONST_DICT(photon_rs485_module_globals, photon_rs485_module_globals_table);

const mp_obj_module_t photon_rs485_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&photon_rs485_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_photon_rs485, photon_rs485_module);
