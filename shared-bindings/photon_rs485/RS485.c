// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/photon_rs485/RS485.h"

#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/util.h"
#include "shared/runtime/context_manager_helpers.h"

#include "py/objproperty.h"
#include "py/runtime.h"

//| class RS485:
//|     """Fast RS485 framing with CRC32 for photon boards.
//|
//|     Use :meth:`add_auto_reply` and :meth:`clear_auto_replies` to configure
//|     request/response pairs handled in C when :meth:`read_frames` is called.
//|     """
//|
//|     def __init__(
//|         self,
//|         tx: microcontroller.Pin,
//|         rx: microcontroller.Pin,
//|         de: microcontroller.Pin,
//|         *,
//|         baudrate: int = 2000000,
//|         device_id: int = 0,
//|         tx_enable_delay_us: int = 12,
//|         rx_buffer_size: int = 16384,
//|         max_payload: int = 256,
//|     ) -> None:
//|         """Create an RS485 framing driver.
//|
//|         :param ~microcontroller.Pin tx: UART transmit pin.
//|         :param ~microcontroller.Pin rx: UART receive pin.
//|         :param ~microcontroller.Pin de: Driver-enable pin.
//|         :param int baudrate: UART baud rate.
//|         :param int device_id: Device ID (0 = host/accept all frames).
//|         :param int tx_enable_delay_us: Delay before/after transmit in microseconds.
//|         :param int rx_buffer_size: Internal receive buffer size in bytes.
//|         :param int max_payload: Maximum payload length in bytes.
//|         """
//|         ...
//|
static mp_obj_t photon_rs485_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    photon_rs485_rs485_obj_t *self = mp_obj_malloc_with_finaliser(photon_rs485_rs485_obj_t, &photon_rs485_rs485_type);

    enum {
        ARG_tx,
        ARG_rx,
        ARG_de,
        ARG_baudrate,
        ARG_device_id,
        ARG_tx_enable_delay_us,
        ARG_rx_buffer_size,
        ARG_max_payload,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_tx, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_rx, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_de, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_baudrate, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 2000000} },
        { MP_QSTR_device_id, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_tx_enable_delay_us, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 12} },
        { MP_QSTR_rx_buffer_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 16384} },
        { MP_QSTR_max_payload, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 256} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const mcu_pin_obj_t *tx = validate_obj_is_free_pin(args[ARG_tx].u_obj, MP_QSTR_tx);
    const mcu_pin_obj_t *rx = validate_obj_is_free_pin(args[ARG_rx].u_obj, MP_QSTR_rx);
    const mcu_pin_obj_t *de = validate_obj_is_free_pin(args[ARG_de].u_obj, MP_QSTR_de);

    if (tx == rx || tx == de || rx == de) {
        mp_raise_ValueError(MP_ERROR_TEXT("pins must be distinct"));
    }

    uint32_t baudrate = (uint32_t)mp_arg_validate_int_min(args[ARG_baudrate].u_int, 1, MP_QSTR_baudrate);
    uint8_t device_id = (uint8_t)mp_arg_validate_int_range(args[ARG_device_id].u_int, 0, 0xFF, MP_QSTR_device_id);
    uint32_t tx_enable_delay_us = (uint32_t)mp_arg_validate_int_min(args[ARG_tx_enable_delay_us].u_int, 0, MP_QSTR_tx_enable_delay_us);
    uint16_t rx_buffer_size = (uint16_t)mp_arg_validate_int_range(args[ARG_rx_buffer_size].u_int, 32, 0xFFFF, MP_QSTR_rx_buffer_size);
    uint16_t max_payload = (uint16_t)mp_arg_validate_int_range(args[ARG_max_payload].u_int, 0, 0xFFFF, MP_QSTR_max_payload);

    common_hal_photon_rs485_construct(self, tx, rx, de, baudrate, device_id,
        max_payload, rx_buffer_size, tx_enable_delay_us);

    return MP_OBJ_FROM_PTR(self);
}

//|     def deinit(self) -> None:
//|         """Deinitialize the driver."""
//|         ...
//|
static mp_obj_t photon_rs485_obj_deinit(mp_obj_t self_in) {
    photon_rs485_rs485_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_photon_rs485_deinit(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(photon_rs485_deinit_obj, photon_rs485_obj_deinit);

static void check_for_deinit(photon_rs485_rs485_obj_t *self) {
    if (common_hal_photon_rs485_deinited(self)) {
        raise_deinited_error();
    }
}

static void photon_rs485_clear_auto_replies(photon_rs485_rs485_obj_t *self) {
    self->auto_reply_entries = mp_obj_new_list(0, NULL);
    self->auto_reply_enabled = false;
}

static void photon_rs485_add_auto_reply(photon_rs485_rs485_obj_t *self,
    uint8_t request_type, uint8_t response_type, mp_obj_t payload_obj) {
    mp_buffer_info_t payload_buf;
    mp_get_buffer_raise(payload_obj, &payload_buf, MP_BUFFER_READ);
    if (payload_buf.len > self->max_payload) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
    }

    mp_obj_t entry_items[3] = {
        MP_OBJ_NEW_SMALL_INT(request_type),
        MP_OBJ_NEW_SMALL_INT(response_type),
        payload_obj,
    };
    mp_obj_list_append(self->auto_reply_entries, mp_obj_new_tuple(3, entry_items));
    self->auto_reply_enabled = true;
}

//|     def clear_auto_replies(self) -> None:
//|         """Disable auto-replies and remove any registered entries."""
//|         ...
//|
static mp_obj_t photon_rs485_obj_clear_auto_replies(mp_obj_t self_in) {
    photon_rs485_rs485_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);
    photon_rs485_clear_auto_replies(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(photon_rs485_clear_auto_replies_obj, photon_rs485_obj_clear_auto_replies);

//|     def send_frame(self, frame_type: int, target_id: int, payload: ReadableBuffer, seq: int, *, ack_timeout_us: int = 2000) -> int:
//|         """Send a framed payload with CRC32.
//|
//|         The ``source_id`` field in the frame header is set automatically from
//|         the driver's ``device_id``.  TX uses DMA on RP2350 for low CPU
//|         overhead.
//|
//|         When ``ack_timeout_us`` is non-zero the driver will tight-poll in C
//|         for a reply frame matching ``frame_type`` (mapped to its expected ACK
//|         type), ``seq``, and ``target_id == self.device_id``.  Returns the
//|         round-trip latency in microseconds (minimum 1) on success, or 0 on
//|         timeout / fire-and-forget.
//|         """
//|         ...
//|
static mp_obj_t photon_rs485_obj_send_frame(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_frame_type, ARG_target_id, ARG_payload, ARG_seq, ARG_ack_timeout_us };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_frame_type, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_target_id, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_payload, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_seq, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_ack_timeout_us, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 2000} },
    };

    photon_rs485_rs485_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    check_for_deinit(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t frame_type = (uint8_t)mp_arg_validate_int_range(args[ARG_frame_type].u_int, 0, 0xFF, MP_QSTR_frame_type);
    uint8_t target_id = (uint8_t)mp_arg_validate_int_range(args[ARG_target_id].u_int, 0, 0xFF, MP_QSTR_target_id);
    uint16_t seq = (uint16_t)mp_arg_validate_int_range(args[ARG_seq].u_int, 0, 0xFFFF, MP_QSTR_seq);
    uint32_t ack_timeout_us = (uint32_t)mp_arg_validate_int_min(args[ARG_ack_timeout_us].u_int, 0, MP_QSTR_ack_timeout_us);

    mp_buffer_info_t payload_buf;
    mp_get_buffer_raise(args[ARG_payload].u_obj, &payload_buf, MP_BUFFER_READ);

    uint32_t result = common_hal_photon_rs485_send_frame(self, frame_type, target_id,
        payload_buf.buf, payload_buf.len, seq, ack_timeout_us);

    return MP_OBJ_NEW_SMALL_INT(result);
}
MP_DEFINE_CONST_FUN_OBJ_KW(photon_rs485_send_frame_obj, 1, photon_rs485_obj_send_frame);

//|     def read_frames(self):
//|         """Return a list of ``(frame_type, target_id, source_id, payload, seq)`` tuples."""
//|         ...
//|
static mp_obj_t photon_rs485_obj_read_frames(mp_obj_t self_in) {
    photon_rs485_rs485_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);
    return common_hal_photon_rs485_read_frames(self);
}
MP_DEFINE_CONST_FUN_OBJ_1(photon_rs485_read_frames_obj, photon_rs485_obj_read_frames);

//|     def add_auto_reply(self, request_type: int, response_type: int, payload: ReadableBuffer) -> None:
//|         """Register an additional auto-reply handler for ``read_frames``."""
//|         ...
//|
static mp_obj_t photon_rs485_obj_add_auto_reply(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_request_type, ARG_response_type, ARG_payload };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_request_type, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_response_type, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_payload, MP_ARG_REQUIRED | MP_ARG_OBJ },
    };

    photon_rs485_rs485_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    check_for_deinit(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t request_type = (uint8_t)mp_arg_validate_int_range(args[ARG_request_type].u_int, 0, 0xFF, MP_QSTR_request_type);
    uint8_t response_type = (uint8_t)mp_arg_validate_int_range(args[ARG_response_type].u_int, 0, 0xFF, MP_QSTR_response_type);

    photon_rs485_add_auto_reply(self, request_type, response_type, args[ARG_payload].u_obj);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(photon_rs485_add_auto_reply_obj, 1, photon_rs485_obj_add_auto_reply);

static const mp_rom_map_elem_t photon_rs485_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&photon_rs485_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&default___exit___obj) },
    { MP_ROM_QSTR(MP_QSTR_send_frame), MP_ROM_PTR(&photon_rs485_send_frame_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_frames), MP_ROM_PTR(&photon_rs485_read_frames_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_auto_replies), MP_ROM_PTR(&photon_rs485_clear_auto_replies_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_auto_reply), MP_ROM_PTR(&photon_rs485_add_auto_reply_obj) },
};
static MP_DEFINE_CONST_DICT(photon_rs485_locals_dict, photon_rs485_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    photon_rs485_rs485_type,
    MP_QSTR_RS485,
    MP_TYPE_FLAG_NONE,
    make_new, photon_rs485_make_new,
    locals_dict, &photon_rs485_locals_dict
    );
