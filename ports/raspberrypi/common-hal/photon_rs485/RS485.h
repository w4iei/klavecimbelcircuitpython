// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>

#include "common-hal/busio/UART.h"
#include "common-hal/microcontroller/Pin.h"
#include "py/obj.h"
#include "hardware/dma.h"

typedef struct {
    mp_obj_base_t base;
    busio_uart_obj_t uart;
    uint8_t de_pin;
    uint8_t device_id;
    uint16_t max_payload;
    uint32_t tx_enable_delay_us;
    uint8_t *buffer;
    size_t buffer_len;
    size_t buffer_capacity;
    uint8_t *tx_buffer;
    size_t tx_buffer_capacity;
    int dma_tx_channel;
    dma_channel_config dma_tx_cfg;
    bool auto_reply_enabled;
    mp_obj_t auto_reply_entries;
} photon_rs485_rs485_obj_t;

void common_hal_photon_rs485_construct(photon_rs485_rs485_obj_t *self,
    const mcu_pin_obj_t *tx, const mcu_pin_obj_t *rx, const mcu_pin_obj_t *de,
    uint32_t baudrate, uint8_t device_id, uint16_t max_payload,
    uint16_t rx_buffer_size, uint32_t tx_enable_delay_us);
bool common_hal_photon_rs485_deinited(photon_rs485_rs485_obj_t *self);
void common_hal_photon_rs485_deinit(photon_rs485_rs485_obj_t *self);
uint32_t common_hal_photon_rs485_send_frame(photon_rs485_rs485_obj_t *self,
    uint8_t frame_type, uint8_t target_id,
    const uint8_t *payload, size_t payload_len, uint16_t seq,
    uint32_t ack_timeout_us);
mp_obj_t common_hal_photon_rs485_read_frames(photon_rs485_rs485_obj_t *self);
