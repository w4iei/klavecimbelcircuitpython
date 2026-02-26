// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: 
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/photon_sensorscan/__init__.h"

#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/microcontroller/Pin.h"

#include "py/binary.h"
#include "py/objarray.h"
#include "py/runtime.h"
#include <string.h>

// photon_sensorscan low-level driver
//
// This module manages multiple TI TLA2518 ADC devices that are attached to the
// same system. The devices are addressed as "banks".
//
// Bank model:
// - One bank == one TLA2518 device and its associated sensor group.
// - Supports up to 16 banks per scanner instance.
// - Each bank is mapped to SPI0 or SPI1 by the constructor-provided
//   bank_spi_bus tuple.
// - Each bank has a dedicated CS pin.
// TLA2518 Datasheet: https://www.ti.com/lit/ds/symlink/tla2518.pdf
//
// The C layer is responsible for SPI/CS ownership, reset/config sequencing, and
// register-level communication. Data-quality decisions are left to Python.

static bool is_initialized;
static busio_spi_obj_t spi0_obj, spi1_obj;
static digitalio_digitalinout_obj_t cs_pins[PHOTON_SENSORSCAN_MAX_BANKS];
static uint8_t bank_count;
static uint8_t slot_count;
MP_REGISTER_ROOT_POINTER(mp_obj_t photonsensorscan_readings_buffer_obj);
static bool readings_buffer_is_bytes;
static size_t readings_buffer_entry_count;
static uint8_t adc_channel_by_slot[PHOTON_SENSORSCAN_MAX_SLOTS];
static uint8_t emitter_mask_by_slot[PHOTON_SENSORSCAN_MAX_SLOTS];
static uint8_t emitter_mask_all_slots;
static bool brown_out_by_bank[PHOTON_SENSORSCAN_MAX_BANKS];
static bool crcerr_fuse_by_bank[PHOTON_SENSORSCAN_MAX_BANKS];
static uint32_t settle_time_us;  // Time to wait after setting an emitter GPIO high, before starting an ADC Capture. 
static uint8_t write_frame_3b[3], transfer_rx_fallback_2b[2], register_read_frame_2b[2];
// Two-byte frame capture buffer for one conversion read.
static uint8_t adc_result_2b[2];
// Dummy transmit bytes used while clocking out ADC conversion data.
static const uint8_t adc_read_clock_2b[2] = {0x00, 0x00};

static const mcu_pin_obj_t *spi0_pin_config[3];
static const mcu_pin_obj_t *spi1_pin_config[3];
static const mcu_pin_obj_t *cs_pin_config[PHOTON_SENSORSCAN_MAX_BANKS];
static uint8_t spi_bus_by_bank[PHOTON_SENSORSCAN_MAX_BANKS];

// Register, bitfield, and opcode definitions
#define TLA25x8_OP_REGISTER_READ (0x10)
#define TLA25x8_OP_REGISTER_WRITE (0x08)
#define TLA25x8_OP_BIT_SET (0x18)
#define TLA25x8_OP_BIT_CLEAR (0x20)
#define TLA25x8_OP_REGISTERS_READ (0x30)
#define TLA25x8_OP_REGISTERS_WRITE (0x28)

#define TLA25x8_SEQ_STATUS_SHIFT (0x6)
#define TLA25x8_SEQ_STATUS_MASK (0x1 << TLA25x8_SEQ_STATUS_SHIFT)
#define TLA25x8_OSR_DONE_SHIFT (0x3)
#define TLA25x8_OSR_DONE_MASK (0x1 << TLA25x8_OSR_DONE_SHIFT)
#define TLA25x8_CRCERR_FUSE_SHIFT (0x2)
#define TLA25x8_CRCERR_FUSE_MASK (0x1 << TLA25x8_CRCERR_FUSE_SHIFT)
#define TLA25x8_BOR_SHIFT (0x0)
#define TLA25x8_BOR_MASK (0x1 << TLA25x8_BOR_SHIFT)

#define TLA25x8_CNVST_SHIFT (0x2)
#define TLA25x8_CNVST_MASK (0x1 << TLA25x8_CNVST_SHIFT)
#define TLA25x8_CH_RST_SHIFT (0x2)
#define TLA25x8_CH_RST_MASK (0x1 << TLA25x8_CH_RST_SHIFT)
#define TLA25x8_CAL_SHIFT (0x1)
#define TLA25x8_CAL_MASK (0x1 << TLA25x8_CAL_SHIFT)
#define TLA25x8_RST_SHIFT (0x0)
#define TLA25x8_RST_MASK (0x1 << TLA25x8_RST_SHIFT)

#define TLA25x8_FIX_PAT_SHIFT (0x7)
#define TLA25x8_FIX_PAT_MASK (0x1 << TLA25x8_FIX_PAT_SHIFT)
#define TLA25x8_APPEND_STATUS_SHIFT (0x4)
#define TLA25x8_APPEND_STATUS_MASK (0x3 << TLA25x8_APPEND_STATUS_SHIFT)
#define TLA25x8_CPOL_CPHA_SHIFT (0x0)
#define TLA25x8_CPOL_CPHA_MASK (0x3 << TLA25x8_CPOL_CPHA_SHIFT)

#define TLA25x8_OSR_SHIFT (0x0)
#define TLA25x8_OSR_MASK (0x7 << TLA25x8_OSR_SHIFT)

#define TLA25x8_OSC_SEL_SHIFT (0x4)
#define TLA25x8_OSC_SEL_MASK (0x1 << TLA25x8_OSC_SEL_SHIFT)
#define TLA25x8_CLK_DIV_SHIFT (0x0)
#define TLA25x8_CLK_DIV_MASK (0xf << TLA25x8_CLK_DIV_SHIFT)

#define TLA25x8_SEQ_START_SHIFT (0x4)
#define TLA25x8_SEQ_START_MASK (0x1 << TLA25x8_SEQ_START_SHIFT)
#define TLA25x8_SEQ_MODE_SHIFT (0x0)
#define TLA25x8_SEQ_MODE_MASK (0x3 << TLA25x8_SEQ_MODE_SHIFT)

#define TLA25x8_MANUAL_CHIPID_SHIFT (0x0)
#define TLA25x8_MANUAL_CHIPID_MASK (0x1 << TLA25x8_MANUAL_CHIPID_SHIFT)

#define TLA25x8_REG_SYSTEM_STATUS (0x00)
#define TLA25x8_REG_GENERAL_CFG (0x01)
#define TLA25x8_REG_DATA_CFG (0x02)
#define TLA25x8_REG_OSR_CONFIG (0x03)
#define TLA25x8_REG_OPMODE_CONFIG (0x04)
#define TLA25x8_REG_PIN_CFG (0x05)
#define TLA25x8_REG_GPIO_CONFIG (0x07)
#define TLA25x8_REG_GPIO_DRIVE_CFG (0x09)
#define TLA25x8_REG_GPO_VALUE (0x0B)
#define TLA25x8_REG_GPI_VALUE (0x0D)
#define TLA25x8_REG_SEQUENCE_CFG (0x10)
#define TLA25x8_REG_CHANNEL_SEL (0x11)
#define TLA25x8_REG_AUTO_SEQ_CH_SEL (0x12)
#define TLA25x8_REG_NUM_REGISTERS (0x13)

#define TLA25x8_RESET_DELAY_US (10000)  // Datasheet says 5ms

#define PHOTON_SENSORSCAN_ERR_SPI_PIN_CLAIM MP_ERROR_TEXT("SPI pin claim failed")
#define PHOTON_SENSORSCAN_ERR_CS_PIN_CLAIM MP_ERROR_TEXT("CS pin claim failed")
#define PHOTON_SENSORSCAN_ERR_SPI_LOCK_CONFIG MP_ERROR_TEXT("SPI lock/config failed")
#define PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET MP_ERROR_TEXT("SPI transfer failed during bank reset/config")
#define PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS MP_ERROR_TEXT("SPI transfer failed during status read")
#define PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME MP_ERROR_TEXT("SPI transfer failed")

static inline busio_spi_obj_t *spi_for_bank(uint8_t bank) {
    return spi_bus_by_bank[bank] == 0 ? &spi0_obj : &spi1_obj;
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

typedef struct {
    bool is_bytes;
    uint8_t *bytes;
    uint16_t *u16;
} readings_buffer_view_t;

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

// Issue a register-read command and read back one byte with CS framed once.
static inline bool read_register1(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs, uint8_t reg, uint8_t *out) {
    register_read_frame_2b[0] = TLA25x8_OP_REGISTER_READ;
    register_read_frame_2b[1] = reg;
    common_hal_digitalio_digitalinout_set_value(cs, false);
    bool ok = common_hal_busio_spi_write(spi, register_read_frame_2b, sizeof(register_read_frame_2b));
    if (ok) {
        ok = common_hal_busio_spi_read(spi, out, 1, 0x00);
    }
    common_hal_digitalio_digitalinout_set_value(cs, true);
    return ok;
}

static inline bool pulse_device_reset(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs) {
    if (!write_register3(spi, cs, TLA25x8_OP_REGISTER_WRITE, TLA25x8_REG_GENERAL_CFG, TLA25x8_RST_MASK)) {
        // Write 0x01 to Register 0x01 (GENERAL_CFG)
        return false;
    }
    delay_microseconds(TLA25x8_RESET_DELAY_US);
    return true;
}

static inline bool bank_write_register(uint8_t bank, uint8_t reg, uint8_t value) {
    return write_register3(
        spi_for_bank(bank),
        cs_for_bank(bank),
        TLA25x8_OP_REGISTER_WRITE,
        reg,
        value
        );
}

static inline bool bank_set_bits(uint8_t bank, uint8_t reg, uint8_t mask) {
    return write_register3(
        spi_for_bank(bank),
        cs_for_bank(bank),
        TLA25x8_OP_BIT_SET,
        reg,
        mask
        );
}

static inline bool bank_clear_bits(uint8_t bank, uint8_t reg, uint8_t mask) {
    return write_register3(
        spi_for_bank(bank),
        cs_for_bank(bank),
        TLA25x8_OP_BIT_CLEAR,
        reg,
        mask
        );
}

static inline bool bank_read_register(uint8_t bank, uint8_t reg, uint8_t *value) {
    return read_register1(spi_for_bank(bank), cs_for_bank(bank), reg, value);
}

static inline bool reset_bank_once(uint8_t bank) {
    return pulse_device_reset(spi_for_bank(bank), cs_for_bank(bank));
}

static inline bool configure_bank_after_reset(uint8_t bank) {
    return bank_clear_bits(bank, TLA25x8_REG_SEQUENCE_CFG, TLA25x8_SEQ_MODE_MASK) &&  // Manual Mode
           bank_set_bits(bank, TLA25x8_REG_PIN_CFG, emitter_mask_all_slots) &&  // Configure GPIO Outputs, pp. 15 
           bank_set_bits(bank, TLA25x8_REG_GPIO_CONFIG, emitter_mask_all_slots) &&
           bank_set_bits(bank, TLA25x8_REG_GPIO_DRIVE_CFG, emitter_mask_all_slots) &&
           bank_clear_bits(bank, TLA25x8_REG_GPO_VALUE, emitter_mask_all_slots) &&  // Unnecessary, but set emitters to off/low
           bank_write_register(bank, TLA25x8_REG_DATA_CFG, 0x00) &&  //Unnecessary
           bank_write_register(bank, TLA25x8_REG_OSR_CONFIG, 0x00); //Unnecessary
}

static inline bool refresh_bank_status(uint8_t bank) {
    uint8_t status = 0;
    if (!bank_read_register(bank, TLA25x8_REG_SYSTEM_STATUS, &status)) {
        return false;
    }
    brown_out_by_bank[bank] = (status & TLA25x8_BOR_MASK) != 0;
    crcerr_fuse_by_bank[bank] = (status & TLA25x8_CRCERR_FUSE_MASK) != 0;
    return true;
}

static inline size_t sensor_value_index(uint8_t bank, uint8_t slot) {
    return ((size_t)bank * slot_count) + slot;
}

static void validate_readings_buffer(
    mp_obj_t readings_buffer, uint8_t bank_count_in, uint8_t slot_count_in,
    bool *is_bytes_out, size_t *value_count_out) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(readings_buffer, &bufinfo, MP_BUFFER_WRITE);

    uint8_t typecode = (uint8_t)(bufinfo.typecode & ~MP_OBJ_ARRAY_TYPECODE_FLAG_RW);
    bool is_bytes = typecode == BYTEARRAY_TYPECODE || typecode == 'B';
    bool is_u16 = typecode == 'H';
    if (!is_bytes && !is_u16) {
        mp_raise_ValueError(MP_ERROR_TEXT("readings_buffer must be bytes or 'H'"));
    }

    size_t value_count = (size_t)bank_count_in * slot_count_in;
    size_t min_len = is_bytes ? value_count * 2 : value_count;
    if (bufinfo.len < min_len) {
        mp_raise_ValueError(MP_ERROR_TEXT("readings_buffer too small"));
    }

    *is_bytes_out = is_bytes;
    *value_count_out = value_count;
}

static void acquire_readings_buffer_view(readings_buffer_view_t *view) {
    mp_obj_t readings_buffer = MP_STATE_VM(photonsensorscan_readings_buffer_obj);
    if (readings_buffer == MP_OBJ_NULL) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("readings_buffer not set"));
    }

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(readings_buffer, &bufinfo, MP_BUFFER_WRITE);
    uint8_t typecode = (uint8_t)(bufinfo.typecode & ~MP_OBJ_ARRAY_TYPECODE_FLAG_RW);
    bool is_bytes = typecode == BYTEARRAY_TYPECODE || typecode == 'B';
    bool is_u16 = typecode == 'H';
    if ((readings_buffer_is_bytes && !is_bytes) ||
        (!readings_buffer_is_bytes && !is_u16)) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("readings_buffer type changed"));
    }

    size_t min_len = readings_buffer_is_bytes ? readings_buffer_entry_count * 2 : readings_buffer_entry_count;
    if (bufinfo.len < min_len) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("readings_buffer resized too small"));
    }

    view->is_bytes = readings_buffer_is_bytes;
    view->bytes = readings_buffer_is_bytes ? (uint8_t *)bufinfo.buf : NULL;
    view->u16 = readings_buffer_is_bytes ? NULL : (uint16_t *)bufinfo.buf;
}

static inline void write_sensor_value(
    readings_buffer_view_t *view, uint8_t bank, uint8_t slot, uint16_t value) {
    size_t index = sensor_value_index(bank, slot);
    if (view->is_bytes) {
        size_t offset = index * 2;
        view->bytes[offset] = (uint8_t)(value & 0xFF);
        view->bytes[offset + 1] = (uint8_t)(value >> 8);
    } else {
        view->u16[index] = value;
    }
}

static bool reset_all_banks_core(void) {
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        common_hal_digitalio_digitalinout_set_value(cs_for_bank(bank), true);  // In case of a stale state
        if (!reset_bank_once(bank)) {
            return false;
        }
    }
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        if (!configure_bank_after_reset(bank)) {
            return false;
        }
    }
    return true;
}

static bool refresh_all_bank_status_core(void) {
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        if (!refresh_bank_status(bank)) {
            return false;
        }
    }
    return true;
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

static void assert_spi_pins_available(
    const mcu_pin_obj_t *spi0_sck, const mcu_pin_obj_t *spi0_mosi, const mcu_pin_obj_t *spi0_miso,
    const mcu_pin_obj_t *spi1_sck, const mcu_pin_obj_t *spi1_mosi, const mcu_pin_obj_t *spi1_miso) {
    if (!common_hal_mcu_pin_is_free(spi0_sck) || !common_hal_mcu_pin_is_free(spi0_mosi) || !common_hal_mcu_pin_is_free(spi0_miso) ||
        !common_hal_mcu_pin_is_free(spi1_sck) || !common_hal_mcu_pin_is_free(spi1_mosi) || !common_hal_mcu_pin_is_free(spi1_miso)) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_PIN_CLAIM);
    }
}

static void assert_cs_pins_available(const mcu_pin_obj_t *const cs_in[PHOTON_SENSORSCAN_MAX_BANKS], uint8_t bank_count_in) {
    for (size_t i = 0; i < bank_count_in; i++) {
        if (!common_hal_mcu_pin_is_free(cs_in[i])) {
            mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_CS_PIN_CLAIM);
        }
    }
}

static void deinit_constructed_buses_and_cs(void) {
    if (!common_hal_busio_spi_deinited(&spi0_obj)) {
        common_hal_busio_spi_deinit(&spi0_obj);
    }
    if (!common_hal_busio_spi_deinited(&spi1_obj)) {
        common_hal_busio_spi_deinit(&spi1_obj);
    }
    for (size_t i = 0; i < PHOTON_SENSORSCAN_MAX_BANKS; i++) {
        if (!common_hal_digitalio_digitalinout_deinited(&cs_pins[i])) {
            common_hal_digitalio_digitalinout_deinit(&cs_pins[i]);
        }
    }
}

static void clear_driver_state(void) {
    is_initialized = false;
    bank_count = 0;
    slot_count = 0;
    MP_STATE_VM(photonsensorscan_readings_buffer_obj) = MP_OBJ_NULL;
    readings_buffer_is_bytes = false;
    readings_buffer_entry_count = 0;
    settle_time_us = 0;
    emitter_mask_all_slots = 0;
    memset(adc_channel_by_slot, 0, sizeof(adc_channel_by_slot));
    memset(emitter_mask_by_slot, 0, sizeof(emitter_mask_by_slot));
    memset(brown_out_by_bank, 0, sizeof(brown_out_by_bank));
    memset(crcerr_fuse_by_bank, 0, sizeof(crcerr_fuse_by_bank));
    memset(spi0_pin_config, 0, sizeof(spi0_pin_config));
    memset(spi1_pin_config, 0, sizeof(spi1_pin_config));
    memset(cs_pin_config, 0, sizeof(cs_pin_config));
    memset(spi_bus_by_bank, 0, sizeof(spi_bus_by_bank));
}

static void store_pin_configuration(
    const mcu_pin_obj_t *spi0_sck, const mcu_pin_obj_t *spi0_mosi, const mcu_pin_obj_t *spi0_miso,
    const mcu_pin_obj_t *spi1_sck, const mcu_pin_obj_t *spi1_mosi, const mcu_pin_obj_t *spi1_miso,
    const mcu_pin_obj_t *const cs_in[PHOTON_SENSORSCAN_MAX_BANKS], uint8_t bank_count_in) {
    spi0_pin_config[0] = spi0_sck;
    spi0_pin_config[1] = spi0_mosi;
    spi0_pin_config[2] = spi0_miso;
    spi1_pin_config[0] = spi1_sck;
    spi1_pin_config[1] = spi1_mosi;
    spi1_pin_config[2] = spi1_miso;
    for (size_t i = 0; i < bank_count_in; i++) {
        cs_pin_config[i] = cs_in[i];
    }
}

static void construct_buses_and_cs(
    const mcu_pin_obj_t *spi0_sck, const mcu_pin_obj_t *spi0_mosi, const mcu_pin_obj_t *spi0_miso,
    const mcu_pin_obj_t *spi1_sck, const mcu_pin_obj_t *spi1_mosi, const mcu_pin_obj_t *spi1_miso,
    const mcu_pin_obj_t *const cs_in[PHOTON_SENSORSCAN_MAX_BANKS], uint8_t bank_count_in) {
    common_hal_busio_spi_construct(&spi0_obj, spi0_sck, spi0_mosi, spi0_miso, false);
    common_hal_busio_spi_construct(&spi1_obj, spi1_sck, spi1_mosi, spi1_miso, false);
    for (size_t i = 0; i < bank_count_in; i++) {
        digitalinout_result_t construct_result =
            common_hal_digitalio_digitalinout_construct(&cs_pins[i], cs_in[i]);
        if (construct_result != DIGITALINOUT_OK) {
            mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_CS_PIN_CLAIM);
        }
        digitalinout_result_t output_result =
            common_hal_digitalio_digitalinout_switch_to_output(&cs_pins[i], true, DRIVE_MODE_PUSH_PULL);
        if (output_result != DIGITALINOUT_OK) {
            mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_CS_PIN_CLAIM);
        }
    }
}

static inline uint16_t read_sensor_core(uint8_t bank, uint8_t slot) {
    // Sweep one emitter/sensor slot in a bank and return its ADC reading.
    busio_spi_obj_t *spi = spi_for_bank(bank);
    digitalio_digitalinout_obj_t *cs = cs_for_bank(bank);
    uint8_t emitter_mask = emitter_mask_by_slot[slot];
    uint8_t adc_channel = adc_channel_by_slot[slot];

    if (emitter_mask != 0 && !write_register3(spi, cs, TLA25x8_OP_BIT_SET, TLA25x8_REG_GPO_VALUE, emitter_mask)) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
    }

    delay_microseconds(settle_time_us);
    // In manual mode, channel switch is decoded on CS rising edge (cycle N), and
    // newly selected channel data are available in cycle N+2.
    // Frame 1: write MANUAL_CHID (select channel)
    // Frame 2: read/discard prior conversion
    // Frame 3: read selected channel conversion
    if (!write_register3(spi, cs, TLA25x8_OP_REGISTER_WRITE, TLA25x8_REG_CHANNEL_SEL, adc_channel & 0x0F) ||
        !transfer2(spi, cs, adc_read_clock_2b, adc_result_2b) ||  // Discard first read to ensure proper channel
        !transfer2(spi, cs, adc_read_clock_2b, adc_result_2b)) {
        if (emitter_mask != 0) {
            (void)write_register3(spi, cs, TLA25x8_OP_BIT_CLEAR, TLA25x8_REG_GPO_VALUE, emitter_mask);
        }
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
    }

    uint16_t value = decode12(adc_result_2b);
    if (emitter_mask != 0 && !write_register3(spi, cs, TLA25x8_OP_BIT_CLEAR, TLA25x8_REG_GPO_VALUE, emitter_mask)) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
    }
    return value;
}

void common_hal_photonsensorscan_init(
    const mcu_pin_obj_t *spi0_sck, const mcu_pin_obj_t *spi0_mosi, const mcu_pin_obj_t *spi0_miso,
    const mcu_pin_obj_t *spi1_sck, const mcu_pin_obj_t *spi1_mosi, const mcu_pin_obj_t *spi1_miso,
    uint8_t bank_count_in,
    const mcu_pin_obj_t *const cs_in[PHOTON_SENSORSCAN_MAX_BANKS],
    const uint8_t bank_spi_bus_in[PHOTON_SENSORSCAN_MAX_BANKS],
    uint8_t slot_count_in, const uint8_t adc_channels_in[PHOTON_SENSORSCAN_MAX_SLOTS], const int8_t emitter_bits_in[PHOTON_SENSORSCAN_MAX_SLOTS],
    mp_obj_t readings_buffer,
    uint32_t settle_us_in, uint32_t baudrate, uint8_t polarity, uint8_t phase) {

    if (bank_count_in < 1 || bank_count_in > PHOTON_SENSORSCAN_MAX_BANKS) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid bank_count"));
    }
    if (slot_count_in > PHOTON_SENSORSCAN_MAX_SLOTS) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many slots"));
    }
    for (size_t i = 0; i < bank_count_in; i++) {
        if (bank_spi_bus_in[i] > 1) {
            mp_raise_ValueError(MP_ERROR_TEXT("bank_spi_bus values must be 0 or 1"));
        }
    }
    bool readings_buffer_is_bytes_in = false;
    size_t readings_value_count = 0;
    validate_readings_buffer(
        readings_buffer, bank_count_in, slot_count_in,
        &readings_buffer_is_bytes_in, &readings_value_count);

    bool need_construct = !is_initialized;
    if (is_initialized) {
        if (spi0_pin_config[0] != spi0_sck || spi0_pin_config[1] != spi0_mosi || spi0_pin_config[2] != spi0_miso ||
            spi1_pin_config[0] != spi1_sck || spi1_pin_config[1] != spi1_mosi || spi1_pin_config[2] != spi1_miso) {
            mp_raise_ValueError(MP_ERROR_TEXT("pins cannot change"));
        }
        if (bank_count_in != bank_count) {
            mp_raise_ValueError(MP_ERROR_TEXT("bank_count cannot change"));
        }
        for (size_t i = 0; i < bank_count; i++) {
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
            deinit_constructed_buses_and_cs();
        }
        is_initialized = false;

        assert_spi_pins_available(spi0_sck, spi0_mosi, spi0_miso, spi1_sck, spi1_mosi, spi1_miso);
        assert_cs_pins_available(cs_in, bank_count_in);
        store_pin_configuration(spi0_sck, spi0_mosi, spi0_miso, spi1_sck, spi1_mosi, spi1_miso, cs_in, bank_count_in);
        construct_buses_and_cs(spi0_sck, spi0_mosi, spi0_miso, spi1_sck, spi1_mosi, spi1_miso, cs_in, bank_count_in);
    }

    if (!ensure_spi_lock_and_config(&spi0_obj, baudrate, polarity, phase) ||
        !ensure_spi_lock_and_config(&spi1_obj, baudrate, polarity, phase)) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_LOCK_CONFIG);
    }

    bank_count = bank_count_in;
    slot_count = slot_count_in;
    settle_time_us = settle_us_in;
    for (size_t i = 0; i < bank_count; i++) {
        spi_bus_by_bank[i] = bank_spi_bus_in[i];
    }
    emitter_mask_all_slots = 0;
    for (size_t i = 0; i < PHOTON_SENSORSCAN_MAX_SLOTS; i++) {
        adc_channel_by_slot[i] = 0;
        emitter_mask_by_slot[i] = 0;
    }
    for (size_t i = 0; i < PHOTON_SENSORSCAN_MAX_BANKS; i++) {
        brown_out_by_bank[i] = false;
        crcerr_fuse_by_bank[i] = false;
    }
    for (size_t i = 0; i < slot_count; i++) {
        adc_channel_by_slot[i] = adc_channels_in[i];
        if (emitter_bits_in[i] >= 0) {
            emitter_mask_by_slot[i] = (uint8_t)(1u << emitter_bits_in[i]);
        }
        emitter_mask_all_slots |= emitter_mask_by_slot[i];
    }

    if (!reset_all_banks_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET);
    }
    if (!refresh_all_bank_status_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }

    MP_STATE_VM(photonsensorscan_readings_buffer_obj) = readings_buffer;
    readings_buffer_is_bytes = readings_buffer_is_bytes_in;
    readings_buffer_entry_count = readings_value_count;
    is_initialized = true;
}

void common_hal_photonsensorscan_deinit(void) {
    deinit_constructed_buses_and_cs();
    clear_driver_state();
}

uint16_t common_hal_photonsensorscan_refresh_sensor(mp_int_t bank_in, mp_int_t slot_in) {
    ensure_initialized();
    uint8_t bank = (uint8_t)mp_arg_validate_int_range(bank_in, 0, bank_count - 1, MP_QSTR_bank);
    if (slot_count == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("no slots configured"));
    }
    uint8_t slot = (uint8_t)mp_arg_validate_int_range(slot_in, 0, slot_count - 1, MP_QSTR_slot);
    readings_buffer_view_t view;
    acquire_readings_buffer_view(&view);
    uint16_t value = read_sensor_core(bank, slot);
    write_sensor_value(&view, bank, slot, value);
    return value;
}

void common_hal_photonsensorscan_refresh_bank(mp_int_t bank_in) {
    ensure_initialized();
    uint8_t bank = (uint8_t)mp_arg_validate_int_range(bank_in, 0, bank_count - 1, MP_QSTR_bank);
    if (slot_count == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("no slots configured"));
    }
    readings_buffer_view_t view;
    acquire_readings_buffer_view(&view);
    for (uint8_t slot = 0; slot < slot_count; slot++) {
        uint16_t value = read_sensor_core(bank, slot);
        write_sensor_value(&view, bank, slot, value);
    }
}

void common_hal_photonsensorscan_refresh_all(void) {
    ensure_initialized();
    if (slot_count == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("no slots configured"));
    }
    readings_buffer_view_t view;
    acquire_readings_buffer_view(&view);
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        for (uint8_t slot = 0; slot < slot_count; slot++) {
            uint16_t value = read_sensor_core(bank, slot);
            write_sensor_value(&view, bank, slot, value);
        }
    }
}

bool common_hal_photonsensorscan_brownout(mp_int_t bank_in) {
    ensure_initialized();
    uint8_t bank = (uint8_t)mp_arg_validate_int_range(bank_in, 0, bank_count - 1, MP_QSTR_bank);
    return brown_out_by_bank[bank];
}

bool common_hal_photonsensorscan_crcerr_fuse(mp_int_t bank_in) {
    ensure_initialized();
    uint8_t bank = (uint8_t)mp_arg_validate_int_range(bank_in, 0, bank_count - 1, MP_QSTR_bank);
    return crcerr_fuse_by_bank[bank];
}

void common_hal_photonsensorscan_reset_device(mp_int_t bank_in) {
    ensure_initialized();
    uint8_t bank = (uint8_t)mp_arg_validate_int_range(bank_in, 0, bank_count - 1, MP_QSTR_bank);
    if (!reset_bank_once(bank) ||
        !configure_bank_after_reset(bank)) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET);
    }
    if (!refresh_bank_status(bank)) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }
}

void common_hal_photonsensorscan_reset_all_banks(void) {
    ensure_initialized();
    if (!reset_all_banks_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET);
    }
    if (!refresh_all_bank_status_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }
}

void common_hal_photonsensorscan_refresh_status(void) {
    ensure_initialized();
    if (!refresh_all_bank_status_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }
}

uint8_t common_hal_photonsensorscan_slot_count(void) {
    return slot_count;
}
