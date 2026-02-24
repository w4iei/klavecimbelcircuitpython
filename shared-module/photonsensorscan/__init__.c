// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2026 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/photonsensorscan/__init__.h"

#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "shared-bindings/microcontroller/__init__.h"

#include "py/binary.h"
#include "py/objarray.h"
#include "py/runtime.h"

static bool is_initialized;
static busio_spi_obj_t spi0_obj, spi1_obj;
static digitalio_digitalinout_obj_t cs_pins[8];
static uint8_t slot_count;
static uint8_t adc_channel_by_slot[8];
static uint8_t emitter_mask_by_slot[8];
static uint8_t emitter_mask_all_slots;
static uint32_t settle_time_us;
// Reused tiny transfer buffers to avoid per-sample stack churn in the hot path.
// write_frame_3b: [opcode, register, value] helper for register writes.
// transfer_rx_fallback_2b: fallback receive scratch when transfer() is unavailable.
static uint8_t write_frame_3b[3], transfer_rx_fallback_2b[2];
// Two-byte frame capture buffer for one conversion read.
static uint8_t adc_result_2b[2];
// Dummy transmit bytes used while clocking out ADC conversion data.
static const uint8_t adc_read_clock_2b[2] = {0x00, 0x00};

static const mcu_pin_obj_t *spi0_pin_config[3];
static const mcu_pin_obj_t *spi1_pin_config[3];
static const mcu_pin_obj_t *cs_pin_config[8];

#define TLA2518_OP_WRITE (0x08)
#define TLA2518_OP_SET (0x18)
#define TLA2518_OP_CLR (0x20)

#define TLA2518_REG_DATA_CFG (0x02)
#define TLA2518_REG_OSR_CFG (0x03)
#define TLA2518_REG_PIN_CFG (0x05)
#define TLA2518_REG_GPIO_CFG (0x07)
#define TLA2518_REG_GPO_DRIVE_CFG (0x09)
#define TLA2518_REG_GPO_VALUE (0x0B)
#define TLA2518_REG_SEQUENCE_CFG (0x10)
#define TLA2518_REG_CHANNEL_SEL (0x11)

#define TLA2518_SEQ_MODE_MASK (0x03)
#define TLA2518_STARTUP_MIN_VALID (1)

static inline busio_spi_obj_t *spi_for_bank(uint8_t bank) {
    return bank < 4 ? &spi0_obj : &spi1_obj;
}

static inline digitalio_digitalinout_obj_t *cs_for_bank(uint8_t bank) {
    return &cs_pins[bank];
}

static inline void delay_microseconds(uint32_t us) {
    if (us > 0) {
        common_hal_mcu_delay_us(us);
    }
}

static inline uint16_t decode12(const uint8_t *buf) {
    return (uint16_t)(((((uint16_t)buf[0] << 8) | buf[1]) >> 4) & 0x0FFF);
}

// Write a 3-byte command frame: opcode + register + value.
static inline bool write_register3(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs, uint8_t op, uint8_t reg, uint8_t val) {
    write_frame_3b[0] = op;
    write_frame_3b[1] = reg;
    write_frame_3b[2] = val;
    common_hal_digitalio_digitalinout_set_value(cs, false);
    bool ok = common_hal_busio_spi_write(spi, write_frame_3b, sizeof(write_frame_3b));
    common_hal_digitalio_digitalinout_set_value(cs, true);
    return ok;
}

// Issue a 2-byte command and read a 2-byte response with CS framing.
static inline bool transfer2(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs, const uint8_t *tx, uint8_t *rx) {
    common_hal_digitalio_digitalinout_set_value(cs, false);
    bool ok = common_hal_busio_spi_transfer(spi, tx, rx, 2);
    if (!ok) {
        ok = common_hal_busio_spi_write(spi, tx, 2);
        if (ok) {
            ok = common_hal_busio_spi_read(spi, transfer_rx_fallback_2b, 2, 0x00);
            if (ok) {
                rx[0] = transfer_rx_fallback_2b[0];
                rx[1] = transfer_rx_fallback_2b[1];
            }
        }
    }
    common_hal_digitalio_digitalinout_set_value(cs, true);
    return ok;
}

static inline void ensure_initialized(void) {
    if (!is_initialized) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("not initialized"));
    }
}

static inline bool ensure_spi_lock_and_config(busio_spi_obj_t *spi, uint32_t baudrate, uint8_t polarity, uint8_t phase) {
    if (!common_hal_busio_spi_has_lock(spi) && !common_hal_busio_spi_try_lock(spi)) {
        return false;
    }
    return common_hal_busio_spi_configure(spi, baudrate, polarity, phase, 8);
}

static inline uint16_t read_sensor_core(uint8_t bank, uint8_t slot) {
    busio_spi_obj_t *spi = spi_for_bank(bank);
    digitalio_digitalinout_obj_t *cs = cs_for_bank(bank);
    uint8_t emitter_mask = emitter_mask_by_slot[slot];
    uint8_t adc_channel = adc_channel_by_slot[slot];

    if (emitter_mask != 0 && !write_register3(spi, cs, TLA2518_OP_SET, TLA2518_REG_GPO_VALUE, emitter_mask)) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("SPI transfer failed"));
    }

    delay_microseconds(settle_time_us);
    // In manual mode, channel switch is decoded on CS rising edge (cycle N), and
    // newly selected channel data are available in cycle N+2.
    // Frame 1: write MANUAL_CHID (select channel)
    // Frame 2: read/discard prior conversion
    // Frame 3: read selected channel conversion
    if (!write_register3(spi, cs, TLA2518_OP_WRITE, TLA2518_REG_CHANNEL_SEL, adc_channel & 0x0F) ||
        !transfer2(spi, cs, adc_read_clock_2b, adc_result_2b) ||
        !transfer2(spi, cs, adc_read_clock_2b, adc_result_2b)) {
        if (emitter_mask != 0) {
            (void)write_register3(spi, cs, TLA2518_OP_CLR, TLA2518_REG_GPO_VALUE, emitter_mask);
        }
        mp_raise_RuntimeError(MP_ERROR_TEXT("SPI transfer failed"));
    }

    uint16_t value = decode12(adc_result_2b);
    if (emitter_mask != 0 && !write_register3(spi, cs, TLA2518_OP_CLR, TLA2518_REG_GPO_VALUE, emitter_mask)) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("SPI transfer failed"));
    }
    return value;
}

void common_hal_photonsensorscan_init(
    const mcu_pin_obj_t *spi0_sck, const mcu_pin_obj_t *spi0_mosi, const mcu_pin_obj_t *spi0_miso,
    const mcu_pin_obj_t *spi1_sck, const mcu_pin_obj_t *spi1_mosi, const mcu_pin_obj_t *spi1_miso,
    const mcu_pin_obj_t *const cs_in[8],
    uint8_t slot_count_in, const uint8_t adc_channels_in[8], const int8_t emitter_bits_in[8],
    uint32_t settle_us_in, uint32_t baudrate, uint8_t polarity, uint8_t phase) {

    if (slot_count_in > 8) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many slots"));
    }

    bool need_construct = !is_initialized;
    if (is_initialized) {
        if (spi0_pin_config[0] != spi0_sck || spi0_pin_config[1] != spi0_mosi || spi0_pin_config[2] != spi0_miso ||
            spi1_pin_config[0] != spi1_sck || spi1_pin_config[1] != spi1_mosi || spi1_pin_config[2] != spi1_miso) {
            mp_raise_ValueError(MP_ERROR_TEXT("pins cannot change"));
        }
        for (size_t i = 0; i < 8; i++) {
            if (cs_pin_config[i] != cs_in[i]) {
                mp_raise_ValueError(MP_ERROR_TEXT("pins cannot change"));
            }
            if (common_hal_digitalio_digitalinout_deinited(&cs_pins[i])) {
                need_construct = true;
            }
        }
        if (common_hal_busio_spi_deinited(&spi0_obj) || common_hal_busio_spi_deinited(&spi1_obj)) {
            need_construct = true;
        }
    }

    if (need_construct) {
        if (is_initialized) {
            if (!common_hal_busio_spi_deinited(&spi0_obj)) {
                common_hal_busio_spi_deinit(&spi0_obj);
            }
            if (!common_hal_busio_spi_deinited(&spi1_obj)) {
                common_hal_busio_spi_deinit(&spi1_obj);
            }
            for (size_t i = 0; i < 8; i++) {
                if (!common_hal_digitalio_digitalinout_deinited(&cs_pins[i])) {
                    common_hal_digitalio_digitalinout_deinit(&cs_pins[i]);
                }
            }
        }

        spi0_pin_config[0] = spi0_sck;
        spi0_pin_config[1] = spi0_mosi;
        spi0_pin_config[2] = spi0_miso;
        spi1_pin_config[0] = spi1_sck;
        spi1_pin_config[1] = spi1_mosi;
        spi1_pin_config[2] = spi1_miso;
        for (size_t i = 0; i < 8; i++) {
            cs_pin_config[i] = cs_in[i];
        }

        common_hal_busio_spi_construct(&spi0_obj, spi0_sck, spi0_mosi, spi0_miso, false);
        common_hal_busio_spi_construct(&spi1_obj, spi1_sck, spi1_mosi, spi1_miso, false);
        for (size_t i = 0; i < 8; i++) {
            common_hal_digitalio_digitalinout_construct(&cs_pins[i], cs_in[i]);
            common_hal_digitalio_digitalinout_switch_to_output(&cs_pins[i], true, DRIVE_MODE_PUSH_PULL);
        }
    }

    if (!ensure_spi_lock_and_config(&spi0_obj, baudrate, polarity, phase) ||
        !ensure_spi_lock_and_config(&spi1_obj, baudrate, polarity, phase)) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("SPI init failed"));
    }

    slot_count = slot_count_in;
    settle_time_us = settle_us_in;
    emitter_mask_all_slots = 0;
    for (size_t i = 0; i < 8; i++) {
        adc_channel_by_slot[i] = 0;
        emitter_mask_by_slot[i] = 0;
    }
    for (size_t i = 0; i < slot_count; i++) {
        adc_channel_by_slot[i] = adc_channels_in[i];
        if (emitter_bits_in[i] >= 0) {
            emitter_mask_by_slot[i] = (uint8_t)(1u << emitter_bits_in[i]);
        }
        emitter_mask_all_slots |= emitter_mask_by_slot[i];
    }

    for (uint8_t bank = 0; bank < 8; bank++) {
        busio_spi_obj_t *spi = spi_for_bank(bank);
        digitalio_digitalinout_obj_t *cs = cs_for_bank(bank);
        common_hal_digitalio_digitalinout_set_value(cs, true);

        if (!write_register3(spi, cs, TLA2518_OP_CLR, TLA2518_REG_SEQUENCE_CFG, TLA2518_SEQ_MODE_MASK) ||
            !write_register3(spi, cs, TLA2518_OP_SET, TLA2518_REG_PIN_CFG, emitter_mask_all_slots) ||
            !write_register3(spi, cs, TLA2518_OP_SET, TLA2518_REG_GPIO_CFG, emitter_mask_all_slots) ||
            !write_register3(spi, cs, TLA2518_OP_SET, TLA2518_REG_GPO_DRIVE_CFG, emitter_mask_all_slots) ||
            !write_register3(spi, cs, TLA2518_OP_CLR, TLA2518_REG_GPO_VALUE, emitter_mask_all_slots) ||
            !write_register3(spi, cs, TLA2518_OP_WRITE, TLA2518_REG_DATA_CFG, 0x00) ||
            !write_register3(spi, cs, TLA2518_OP_WRITE, TLA2518_REG_OSR_CFG, 0x00)) {
            mp_raise_RuntimeError(MP_ERROR_TEXT("SPI init failed"));
        }
    }

    // Detect silent startup failures early: require at least one realistic
    // non-trivial reading when emitters are configured.
    if (slot_count > 0 && emitter_mask_all_slots != 0) {
        uint16_t startup_max = 0;
        for (uint8_t bank = 0; bank < 8; bank++) {
            for (uint8_t slot = 0; slot < slot_count; slot++) {
                uint16_t sample = read_sensor_core(bank, slot);
                if (sample > startup_max) {
                    startup_max = sample;
                }
            }
        }
        if (startup_max < TLA2518_STARTUP_MIN_VALID) {
            mp_raise_RuntimeError(MP_ERROR_TEXT("ADC startup read too low"));
        }
    }

    is_initialized = true;
}

uint16_t common_hal_photonsensorscan_read_sensor(mp_int_t bank_in, mp_int_t slot_in) {
    ensure_initialized();
    uint8_t bank = (uint8_t)mp_arg_validate_int_range(bank_in, 0, 7, MP_QSTR_bank);
    if (slot_count == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("no slots configured"));
    }
    uint8_t slot = (uint8_t)mp_arg_validate_int_range(slot_in, 0, slot_count - 1, MP_QSTR_slot);
    return read_sensor_core(bank, slot);
}

void common_hal_photonsensorscan_scan_bank_into(mp_int_t bank_in, mp_obj_t out) {
    ensure_initialized();
    uint8_t bank = (uint8_t)mp_arg_validate_int_range(bank_in, 0, 7, MP_QSTR_bank);

    mp_buffer_info_t bufinfo;
    bool has_buffer = mp_get_buffer(out, &bufinfo, MP_BUFFER_WRITE);
    bool is_byte_buffer = false;
    if (has_buffer) {
        uint8_t typecode = (uint8_t)(bufinfo.typecode & ~MP_OBJ_ARRAY_TYPECODE_FLAG_RW);
        is_byte_buffer = typecode == BYTEARRAY_TYPECODE || typecode == 'B';
    }

    if (is_byte_buffer) {
        size_t required = (size_t)slot_count * 2;
        if (bufinfo.len < required) {
            mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
        }
        uint8_t *raw = (uint8_t *)bufinfo.buf;
        for (uint8_t slot = 0; slot < slot_count; slot++) {
            uint16_t value = read_sensor_core(bank, slot);
            size_t offset = (size_t)slot * 2;
            raw[offset] = (uint8_t)(value & 0xFF);
            raw[offset + 1] = (uint8_t)(value >> 8);
        }
        return;
    }

    for (uint8_t slot = 0; slot < slot_count; slot++) {
        uint16_t value = read_sensor_core(bank, slot);
        mp_obj_subscr(out, MP_OBJ_NEW_SMALL_INT(slot), MP_OBJ_NEW_SMALL_INT(value));
    }
}

uint8_t common_hal_photonsensorscan_slot_count(void) {
    return slot_count;
}
