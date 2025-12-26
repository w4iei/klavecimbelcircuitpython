// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "common-hal/photon_rs485/RS485.h"

#include <string.h>

#include "shared-bindings/busio/UART.h"
#include "shared-bindings/microcontroller/__init__.h"
#include "supervisor/shared/tick.h"

#include "py/mperrno.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/stream.h"

#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/uart.h"

#define NO_PIN 0xff

#define FRAME_PREAMBLE_LEN (4)
#define FRAME_HEADER_LEN (10)
#define FRAME_TRAILER_LEN (4)

#define FRAME_TYPE_PING ((uint8_t)'P')
#define FRAME_TYPE_PONG ((uint8_t)'O')
#define FRAME_TYPE_DATA_REQ ((uint8_t)'R')
#define FRAME_TYPE_DATA_RESP ((uint8_t)'D')
#define FRAME_TYPE_STATS_REQ ((uint8_t)'S')
#define FRAME_TYPE_STATS_RESP ((uint8_t)'s')

static const uint8_t photon_rs485_preamble[FRAME_PREAMBLE_LEN] = { 0xA5, 0x5A, 0xC3, 0x3C };

static const uint32_t photon_rs485_crc32_table[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
    0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
    0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
};

static uint32_t photon_rs485_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        crc = photon_rs485_crc32_table[crc & 0x0F] ^ (crc >> 4);
        crc = photon_rs485_crc32_table[crc & 0x0F] ^ (crc >> 4);
    }
    return crc ^ 0xFFFFFFFFu;
}

static int32_t photon_rs485_find_preamble(const uint8_t *buffer, size_t start, size_t len) {
    for (size_t i = start; i + FRAME_PREAMBLE_LEN <= len; ++i) {
        if (memcmp(buffer + i, photon_rs485_preamble, FRAME_PREAMBLE_LEN) == 0) {
            return (int32_t)i;
        }
    }
    return -1;
}

static void photon_rs485_fill_rx_buffer(photon_rs485_rs485_obj_t *self) {
    size_t available = common_hal_busio_uart_rx_characters_available(&self->uart);
    while (available > 0 && self->buffer_len < self->buffer_capacity) {
        size_t space = self->buffer_capacity - self->buffer_len;
        size_t to_read = available < space ? available : space;
        int errcode = 0;
        size_t read_len = common_hal_busio_uart_read(&self->uart,
            self->buffer + self->buffer_len, to_read, &errcode);
        if (read_len == MP_STREAM_ERROR) {
            if (errcode != EAGAIN) {
                mp_raise_OSError(errcode);
            }
            break;
        }
        self->buffer_len += read_len;
        if (read_len < to_read) {
            break;
        }
        available = common_hal_busio_uart_rx_characters_available(&self->uart);
    }
}

static size_t photon_rs485_prepare_frame(photon_rs485_rs485_obj_t *self,
    uint8_t frame_type, uint8_t target_id,
    const uint8_t *payload, size_t payload_len, uint16_t seq,
    bool payload_inplace) {

    if (payload_len > self->max_payload) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
    }

    size_t total_len = FRAME_HEADER_LEN + payload_len + FRAME_TRAILER_LEN;
    if (total_len > self->tx_buffer_capacity) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
    }

    uint8_t *buf = self->tx_buffer;
    memcpy(buf, photon_rs485_preamble, FRAME_PREAMBLE_LEN);
    buf[4] = frame_type;
    buf[5] = target_id;
    buf[6] = (uint8_t)(payload_len & 0xFF);
    buf[7] = (uint8_t)((payload_len >> 8) & 0xFF);
    buf[8] = (uint8_t)(seq & 0xFF);
    buf[9] = (uint8_t)((seq >> 8) & 0xFF);

    uint8_t *payload_dst = buf + FRAME_HEADER_LEN;
    if (!payload_inplace && payload_len > 0) {
        memcpy(payload_dst, payload, payload_len);
    }

    uint32_t crc = photon_rs485_crc32(payload_dst, payload_len);
    buf[FRAME_HEADER_LEN + payload_len] = (uint8_t)(crc & 0xFF);
    buf[FRAME_HEADER_LEN + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);
    buf[FRAME_HEADER_LEN + payload_len + 2] = (uint8_t)((crc >> 16) & 0xFF);
    buf[FRAME_HEADER_LEN + payload_len + 3] = (uint8_t)((crc >> 24) & 0xFF);

    return total_len;
}

static void photon_rs485_send_prepared(photon_rs485_rs485_obj_t *self, size_t total_len) {
    gpio_put(self->de_pin, true);
    if (self->tx_enable_delay_us > 0) {
        common_hal_mcu_delay_us(self->tx_enable_delay_us);
    }

    dma_channel_configure(self->dma_tx_channel, &self->dma_tx_cfg,
        &uart_get_hw(self->uart.uart)->dr, self->tx_buffer, total_len, true);
    dma_channel_wait_for_finish_blocking(self->dma_tx_channel);
    uart_tx_wait_blocking(self->uart.uart);

    if (self->tx_enable_delay_us > 0) {
        common_hal_mcu_delay_us(self->tx_enable_delay_us);
    }
    gpio_put(self->de_pin, false);
}

static size_t photon_rs485_build_values_payload(photon_rs485_rs485_obj_t *self, mp_obj_t values_obj) {
    mp_buffer_info_t bufinfo;
    if (mp_get_buffer(values_obj, &bufinfo, MP_BUFFER_READ)) {
        if (bufinfo.len > self->max_payload) {
            mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
        }
        memcpy(self->tx_buffer + FRAME_HEADER_LEN, bufinfo.buf, bufinfo.len);
        return bufinfo.len;
    }

    size_t values_len = 0;
    mp_obj_t *values_items = NULL;
    mp_obj_get_array(values_obj, &values_len, &values_items);
    size_t payload_len = values_len * 2;
    if (payload_len > self->max_payload) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
    }

    uint8_t *payload = self->tx_buffer + FRAME_HEADER_LEN;
    for (size_t i = 0; i < values_len; i++) {
        uint16_t value = (uint16_t)mp_obj_get_int(values_items[i]);
        payload[i * 2] = (uint8_t)(value & 0xFF);
        payload[i * 2 + 1] = (uint8_t)((value >> 8) & 0xFF);
    }
    return payload_len;
}

static uint16_t photon_rs485_stats_rate_tenths(mp_obj_t scan_times_obj) {
    if (scan_times_obj == mp_const_none) {
        return 0;
    }

    size_t times_len = 0;
    mp_obj_t *times_items = NULL;
    mp_obj_get_array(scan_times_obj, &times_len, &times_items);
    if (times_len == 0) {
        return 0;
    }

    mp_float_t first = mp_obj_get_float(times_items[0]);
    mp_float_t now = (mp_float_t)(supervisor_ticks_ms64() / 1000.0f);
    mp_float_t elapsed = now - first;
    if (elapsed < 0.001f) {
        elapsed = 0.001f;
    }
    mp_float_t rate = (mp_float_t)times_len / elapsed;
    mp_int_t rate_tenths = (mp_int_t)(rate * 10.0f + 0.5f);
    if (rate_tenths < 0) {
        rate_tenths = 0;
    }
    if (rate_tenths > 0xFFFF) {
        rate_tenths = 0xFFFF;
    }
    return (uint16_t)rate_tenths;
}

void common_hal_photon_rs485_construct(photon_rs485_rs485_obj_t *self,
    const mcu_pin_obj_t *tx, const mcu_pin_obj_t *rx, const mcu_pin_obj_t *de,
    uint32_t baudrate, uint8_t device_id, uint16_t max_payload,
    uint16_t rx_buffer_size, uint32_t tx_enable_delay_us) {

    self->dma_tx_channel = -1;
    self->tx_buffer = NULL;
    self->buffer = NULL;

    size_t min_frame_len = FRAME_HEADER_LEN + (size_t)max_payload + FRAME_TRAILER_LEN;
    if (rx_buffer_size < min_frame_len) {
        mp_raise_ValueError(MP_ERROR_TEXT("rx_buffer_size too small"));
    }

    self->buffer = (uint8_t *)m_malloc(rx_buffer_size);
    if (self->buffer == NULL) {
        m_malloc_fail(rx_buffer_size);
    }

    size_t tx_buffer_size = FRAME_HEADER_LEN + (size_t)max_payload + FRAME_TRAILER_LEN;
    self->tx_buffer = (uint8_t *)m_malloc(tx_buffer_size);
    if (self->tx_buffer == NULL) {
        m_free(self->buffer);
        self->buffer = NULL;
        m_malloc_fail(tx_buffer_size);
    }

    self->buffer_len = 0;
    self->buffer_capacity = rx_buffer_size;
    self->tx_buffer_capacity = tx_buffer_size;
    self->max_payload = max_payload;
    self->device_id = device_id;
    self->tx_enable_delay_us = tx_enable_delay_us;
    self->de_pin = de->number;

    common_hal_busio_uart_construct(&self->uart, tx, rx,
        NULL, NULL, NULL, false,
        baudrate, 8, BUSIO_UART_PARITY_NONE, 1,
        0, rx_buffer_size, NULL, false);

    claim_pin(de);

    gpio_init(self->de_pin);
    gpio_disable_pulls(self->de_pin);
    gpio_put(self->de_pin, false);
    gpio_set_dir(self->de_pin, GPIO_OUT);

    self->dma_tx_channel = dma_claim_unused_channel(true);
    self->dma_tx_cfg = dma_channel_get_default_config(self->dma_tx_channel);
    channel_config_set_read_increment(&self->dma_tx_cfg, true);
    channel_config_set_write_increment(&self->dma_tx_cfg, false);
    channel_config_set_transfer_data_size(&self->dma_tx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&self->dma_tx_cfg, UART_DREQ_NUM(self->uart.uart, true));
}

bool common_hal_photon_rs485_deinited(photon_rs485_rs485_obj_t *self) {
    return common_hal_busio_uart_deinited(&self->uart);
}

void common_hal_photon_rs485_deinit(photon_rs485_rs485_obj_t *self) {
    if (common_hal_photon_rs485_deinited(self)) {
        return;
    }

    common_hal_busio_uart_deinit(&self->uart);

    if (self->dma_tx_channel >= 0) {
        dma_channel_abort(self->dma_tx_channel);
        dma_channel_unclaim(self->dma_tx_channel);
        self->dma_tx_channel = -1;
    }

    if (self->de_pin != NO_PIN) {
        reset_pin_number(self->de_pin);
        self->de_pin = NO_PIN;
    }

    if (self->buffer != NULL) {
        m_free(self->buffer);
        self->buffer = NULL;
        self->buffer_len = 0;
        self->buffer_capacity = 0;
    }

    if (self->tx_buffer != NULL) {
        m_free(self->tx_buffer);
        self->tx_buffer = NULL;
        self->tx_buffer_capacity = 0;
    }
}

void common_hal_photon_rs485_send_frame(photon_rs485_rs485_obj_t *self,
    uint8_t frame_type, uint8_t target_id,
    const uint8_t *payload, size_t payload_len, uint16_t seq) {

    size_t total_len = photon_rs485_prepare_frame(self, frame_type, target_id,
        payload, payload_len, seq, false);
    photon_rs485_send_prepared(self, total_len);
}

mp_obj_t common_hal_photon_rs485_read_frames(photon_rs485_rs485_obj_t *self) {
    photon_rs485_fill_rx_buffer(self);

    mp_obj_t frames = mp_obj_new_list(0, NULL);
    size_t offset = 0;
    size_t max_frame_len = FRAME_HEADER_LEN + (size_t)self->max_payload + FRAME_TRAILER_LEN;

    while (true) {
        size_t remaining = self->buffer_len - offset;
        if (remaining < FRAME_HEADER_LEN) {
            break;
        }

        int32_t preamble_index = photon_rs485_find_preamble(self->buffer, offset, self->buffer_len);
        if (preamble_index < 0) {
            if (self->buffer_len > (FRAME_PREAMBLE_LEN - 1)) {
                offset = self->buffer_len - (FRAME_PREAMBLE_LEN - 1);
            }
            break;
        }

        if ((size_t)preamble_index > offset) {
            offset = (size_t)preamble_index;
            remaining = self->buffer_len - offset;
            if (remaining < FRAME_HEADER_LEN) {
                break;
            }
        }

        uint8_t frame_type = self->buffer[offset + 4];
        uint8_t target_id = self->buffer[offset + 5];
        uint16_t length = (uint16_t)(self->buffer[offset + 6] | (self->buffer[offset + 7] << 8));
        uint16_t seq = (uint16_t)(self->buffer[offset + 8] | (self->buffer[offset + 9] << 8));

        size_t total_len = FRAME_HEADER_LEN + (size_t)length + FRAME_TRAILER_LEN;
        if (length > self->max_payload || total_len > max_frame_len) {
            offset += 1;
            continue;
        }

        if (remaining < total_len) {
            break;
        }

        size_t payload_start = offset + FRAME_HEADER_LEN;
        size_t crc_start = payload_start + length;

        uint32_t crc_rx = (uint32_t)(self->buffer[crc_start] |
            (self->buffer[crc_start + 1] << 8) |
            (self->buffer[crc_start + 2] << 16) |
            (self->buffer[crc_start + 3] << 24));
        uint32_t crc_calc = photon_rs485_crc32(self->buffer + payload_start, length);
        if (crc_rx != crc_calc) {
            offset += 1;
            continue;
        }

        bool accept = (self->device_id == 0) || (target_id == 0) || (target_id == self->device_id);
        if (accept) {
            mp_obj_t payload_obj = mp_obj_new_bytes(self->buffer + payload_start, length);
            mp_obj_t items[4] = {
                MP_OBJ_NEW_SMALL_INT(frame_type),
                MP_OBJ_NEW_SMALL_INT(target_id),
                payload_obj,
                MP_OBJ_NEW_SMALL_INT(seq),
            };
            mp_obj_list_append(frames, mp_obj_new_tuple(4, items));
        }

        offset += total_len;
    }

    if (offset > 0) {
        if (offset < self->buffer_len) {
            memmove(self->buffer, self->buffer + offset, self->buffer_len - offset);
            self->buffer_len -= offset;
        } else {
            self->buffer_len = 0;
        }
    }

    return frames;
}

mp_int_t common_hal_photon_rs485_process(photon_rs485_rs485_obj_t *self,
    mp_obj_t latest_values, mp_obj_t scan_times) {

    photon_rs485_fill_rx_buffer(self);

    mp_int_t replies = 0;
    size_t offset = 0;
    size_t max_frame_len = FRAME_HEADER_LEN + (size_t)self->max_payload + FRAME_TRAILER_LEN;

    while (true) {
        size_t remaining = self->buffer_len - offset;
        if (remaining < FRAME_HEADER_LEN) {
            break;
        }

        int32_t preamble_index = photon_rs485_find_preamble(self->buffer, offset, self->buffer_len);
        if (preamble_index < 0) {
            if (self->buffer_len > (FRAME_PREAMBLE_LEN - 1)) {
                offset = self->buffer_len - (FRAME_PREAMBLE_LEN - 1);
            }
            break;
        }

        if ((size_t)preamble_index > offset) {
            offset = (size_t)preamble_index;
            remaining = self->buffer_len - offset;
            if (remaining < FRAME_HEADER_LEN) {
                break;
            }
        }

        uint8_t frame_type = self->buffer[offset + 4];
        uint8_t target_id = self->buffer[offset + 5];
        uint16_t length = (uint16_t)(self->buffer[offset + 6] | (self->buffer[offset + 7] << 8));
        uint16_t seq = (uint16_t)(self->buffer[offset + 8] | (self->buffer[offset + 9] << 8));

        size_t total_len = FRAME_HEADER_LEN + (size_t)length + FRAME_TRAILER_LEN;
        if (length > self->max_payload || total_len > max_frame_len) {
            offset += 1;
            continue;
        }

        if (remaining < total_len) {
            break;
        }

        size_t payload_start = offset + FRAME_HEADER_LEN;
        size_t crc_start = payload_start + length;

        uint32_t crc_rx = (uint32_t)(self->buffer[crc_start] |
            (self->buffer[crc_start + 1] << 8) |
            (self->buffer[crc_start + 2] << 16) |
            (self->buffer[crc_start + 3] << 24));
        uint32_t crc_calc = photon_rs485_crc32(self->buffer + payload_start, length);
        if (crc_rx != crc_calc) {
            offset += 1;
            continue;
        }

        bool accept = (self->device_id == 0) || (target_id == 0) || (target_id == self->device_id);
        bool addressed = (target_id == self->device_id || target_id == 0);
        if (accept && addressed) {
            if (frame_type == FRAME_TYPE_PING) {
                size_t reply_len = photon_rs485_prepare_frame(self, FRAME_TYPE_PONG,
                    self->device_id, NULL, 0, seq, false);
                photon_rs485_send_prepared(self, reply_len);
                replies += 1;
            } else if (frame_type == FRAME_TYPE_DATA_REQ) {
                size_t payload_len = photon_rs485_build_values_payload(self, latest_values);
                size_t reply_len = photon_rs485_prepare_frame(self, FRAME_TYPE_DATA_RESP,
                    self->device_id, self->tx_buffer + FRAME_HEADER_LEN, payload_len, seq, true);
                photon_rs485_send_prepared(self, reply_len);
                replies += 1;
            } else if (frame_type == FRAME_TYPE_STATS_REQ) {
                uint16_t rate_tenths = photon_rs485_stats_rate_tenths(scan_times);
                uint8_t *payload = self->tx_buffer + FRAME_HEADER_LEN;
                payload[0] = (uint8_t)(rate_tenths & 0xFF);
                payload[1] = (uint8_t)((rate_tenths >> 8) & 0xFF);
                size_t reply_len = photon_rs485_prepare_frame(self, FRAME_TYPE_STATS_RESP,
                    self->device_id, payload, 2, seq, true);
                photon_rs485_send_prepared(self, reply_len);
                replies += 1;
            }
        }

        offset += total_len;
    }

    if (offset > 0) {
        if (offset < self->buffer_len) {
            memmove(self->buffer, self->buffer + offset, self->buffer_len - offset);
            self->buffer_len -= offset;
        } else {
            self->buffer_len = 0;
        }
    }

    return replies;
}
