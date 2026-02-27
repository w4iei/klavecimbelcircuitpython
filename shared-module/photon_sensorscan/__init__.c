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
#include "supervisor/shared/tick.h"
#include <string.h>

#if defined(PICO_RP2350)
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#endif

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
static busio_spi_obj_t spi_objs[2];
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
// Dummy transmit bytes used while clocking out ADC conversion data.
static const uint8_t adc_read_clock_2b[2] = {0x00, 0x00};

static const mcu_pin_obj_t *spi_pin_config[2][3];
static bool spi_bus_in_use[2];
static const mcu_pin_obj_t *cs_pin_config[PHOTON_SENSORSCAN_MAX_BANKS];
static uint8_t spi_bus_by_bank[PHOTON_SENSORSCAN_MAX_BANKS];
static photonsensorscan_debug_state_t debug_state;
static uint32_t spi_runtime_baudrate;
static uint8_t spi_runtime_polarity;
static uint8_t spi_runtime_phase;

#if defined(PICO_RP2350)
static inline bool rp_gpio_out(uint8_t gpio) {
    return (sio_hw->gpio_out >> gpio) & 1u;
}

static inline bool rp_gpio_oe(uint8_t gpio) {
    return (sio_hw->gpio_oe >> gpio) & 1u;
}

static inline bool rp_gpio_in(uint8_t gpio) {
    return (sio_hw->gpio_in >> gpio) & 1u;
}
#endif

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

#define PHOTON_SENSORSCAN_ERR_SPI_PIN_CLAIM MP_ERROR_TEXT("SPI pin claim failed")
#define PHOTON_SENSORSCAN_ERR_CS_PIN_CLAIM MP_ERROR_TEXT("CS pin claim failed")
#define PHOTON_SENSORSCAN_ERR_SPI_LOCK_CONFIG MP_ERROR_TEXT("SPI lock/config failed")
#define PHOTON_SENSORSCAN_ERR_SPI_FLUSH MP_ERROR_TEXT("SPI flush failed")
#define PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET MP_ERROR_TEXT("SPI transfer failed during bank reset/config")
#define PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS MP_ERROR_TEXT("SPI transfer failed during status read")
#define PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME MP_ERROR_TEXT("SPI transfer failed")

static const uint8_t spi_flush_clock_bytes[16] = {0}; // 16 * 8 = 128 clocks

enum {
    PHOTON_SENSORSCAN_DEBUG_ERR_NONE = 0,
    PHOTON_SENSORSCAN_DEBUG_ERR_INIT_SPI_LOCK = 1,
    PHOTON_SENSORSCAN_DEBUG_ERR_SPI_FLUSH_CONFIG = 2,
    PHOTON_SENSORSCAN_DEBUG_ERR_SPI_FLUSH_WRITE = 3,
    PHOTON_SENSORSCAN_DEBUG_ERR_RESET_BANK = 10,
    PHOTON_SENSORSCAN_DEBUG_ERR_CONFIGURE_BANK = 11,
    PHOTON_SENSORSCAN_DEBUG_ERR_REFRESH_STATUS = 12,
    PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_SET_EMITTER = 20,
    PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_SELECT_CHANNEL = 21,
    PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_PRIME_READ = 22,
    PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_DATA_READ = 23,
    PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_CLEAR_EMITTER = 24,
};

static inline busio_spi_obj_t *spi_for_bus(uint8_t bus) {
    return &spi_objs[bus];
}

static inline busio_spi_obj_t *spi_for_bank(uint8_t bank) {
    return spi_for_bus(spi_bus_by_bank[bank]);
}

static inline uint8_t debug_spi_index(busio_spi_obj_t *spi) {
    if (spi == spi_for_bus(0)) {
        return 0;
    }
    if (spi == spi_for_bus(1)) {
        return 1;
    }
    return 0xFF;
}

static inline void debug_set_error(uint32_t code) {
    debug_state.last_error = code;
}

static inline void debug_set_bank(uint8_t bank) {
    debug_state.last_bank = bank;
    debug_state.last_bus = spi_bus_by_bank[bank];
}

static inline void debug_note_xfer(busio_spi_obj_t *spi, bool ok) {
    uint8_t bus = debug_spi_index(spi);
    if (bus > 1) {
        return;
    }
    debug_state.last_bus = bus;
    if (bus == 0) {
        debug_state.spi0_xfers++;
        if (!ok) {
            debug_state.spi0_fail++;
        }
    } else {
        debug_state.spi1_xfers++;
        if (!ok) {
            debug_state.spi1_fail++;
        }
    }
}

static inline void debug_store_last_rx(const uint8_t *rx, size_t len) {
    debug_state.last_rx_len = len > 0xFFFF ? 0xFFFF : (uint16_t)len;
    memset(debug_state.last_rx, 0, sizeof(debug_state.last_rx));
    size_t copy_len = len > sizeof(debug_state.last_rx) ? sizeof(debug_state.last_rx) : len;
    memcpy(debug_state.last_rx, rx, copy_len);
}

static inline void debug_store_bank_sanity_register(
    uint8_t bank, uint8_t *values, uint16_t *valid_mask, bool ok, uint8_t value) {
    uint16_t bank_bit = (uint16_t)1u << bank;
    if (ok) {
        values[bank] = value;
        *valid_mask |= bank_bit;
    } else {
        values[bank] = 0;
        *valid_mask &= (uint16_t)~bank_bit;
    }
}

static void debug_reset_state(void) {
    memset(&debug_state, 0, sizeof(debug_state));
    debug_state.last_bank = 0xFF;
    debug_state.last_bus = 0xFF;
    debug_state.last_cs_bank = 0xFF;
    debug_state.last_cs_direction = 0xFF;
    debug_state.last_cs_out = 0xFF;
    debug_state.cs_funcsel = 0xFF;
    debug_state.expected_spi_funcsel = 0xFF;
    debug_state.expected_sio_funcsel = 0xFF;
    debug_state.cs_pad_pue = 0xFF;
    debug_state.cs_pad_pde = 0xFF;
    memset(debug_state.sck_funcsel, 0xFF, sizeof(debug_state.sck_funcsel));
    memset(debug_state.mosi_funcsel, 0xFF, sizeof(debug_state.mosi_funcsel));
    memset(debug_state.miso_funcsel, 0xFF, sizeof(debug_state.miso_funcsel));
#if defined(PICO_RP2350)
    debug_state.expected_spi_funcsel = (uint8_t)GPIO_FUNC_SPI;
    debug_state.expected_sio_funcsel = (uint8_t)GPIO_FUNC_SIO;
#endif
}

static inline void delay_microseconds(uint32_t us);
static inline bool ensure_spi_lock_and_config(busio_spi_obj_t *spi, uint32_t baudrate, uint8_t polarity, uint8_t phase);

static inline digitalio_digitalinout_obj_t *cs_for_bank(uint8_t bank) {
    return &cs_pins[bank];
}

static inline uint8_t debug_cs_index(digitalio_digitalinout_obj_t *cs) {
    if (cs >= &cs_pins[0] && cs < &cs_pins[PHOTON_SENSORSCAN_MAX_BANKS]) {
        return (uint8_t)(cs - &cs_pins[0]);
    }
    return 0xFF;
}

static inline void debug_capture_cs_mux_state(digitalio_digitalinout_obj_t *cs) {
    if (cs == NULL || cs->pin == NULL) {
        debug_state.cs_funcsel = 0xFF;
        debug_state.cs_pad_pue = 0xFF;
        debug_state.cs_pad_pde = 0xFF;
        return;
    }
    #if defined(PICO_RP2350)
    uint8_t gpio = cs->pin->number;
    uint8_t funcsel = (uint8_t)gpio_get_function(gpio);
    uint8_t expected_sio_funcsel = (uint8_t)GPIO_FUNC_SIO;
    debug_state.cs_funcsel = funcsel;
    debug_state.expected_sio_funcsel = expected_sio_funcsel;
    debug_state.cs_pad_pue = gpio_is_pulled_up(gpio) ? 1 : 0;
    debug_state.cs_pad_pde = gpio_is_pulled_down(gpio) ? 1 : 0;
    if (funcsel != expected_sio_funcsel) {
        debug_state.cs_funcsel_assert_fail++;
    }
    #else
    debug_state.cs_funcsel = 0xFF;
    debug_state.cs_pad_pue = 0xFF;
    debug_state.cs_pad_pde = 0xFF;
    #endif
}

static inline void debug_capture_spi_mux_state(uint8_t bus) {
    if (bus > 1 || !spi_bus_in_use[bus]) {
        return;
    }
    #if defined(PICO_RP2350)
    bool mismatch = false;
    uint8_t expected_spi_funcsel = (uint8_t)GPIO_FUNC_SPI;
    const mcu_pin_obj_t *sck = spi_pin_config[bus][0];
    const mcu_pin_obj_t *mosi = spi_pin_config[bus][1];
    const mcu_pin_obj_t *miso = spi_pin_config[bus][2];
    debug_state.expected_spi_funcsel = expected_spi_funcsel;

    debug_state.sck_funcsel[bus] = sck == NULL ? 0xFF : (uint8_t)gpio_get_function(sck->number);
    debug_state.mosi_funcsel[bus] = mosi == NULL ? 0xFF : (uint8_t)gpio_get_function(mosi->number);
    debug_state.miso_funcsel[bus] = miso == NULL ? 0xFF : (uint8_t)gpio_get_function(miso->number);

    if (sck != NULL && debug_state.sck_funcsel[bus] != expected_spi_funcsel) {
        mismatch = true;
    }
    if (mosi != NULL && debug_state.mosi_funcsel[bus] != expected_spi_funcsel) {
        mismatch = true;
    }
    if (miso != NULL && debug_state.miso_funcsel[bus] != expected_spi_funcsel) {
        mismatch = true;
    }
    if (mismatch) {
        debug_state.spi_funcsel_assert_fail[bus]++;
    }
    #else
    debug_state.sck_funcsel[bus] = 0xFF;
    debug_state.mosi_funcsel[bus] = 0xFF;
    debug_state.miso_funcsel[bus] = 0xFF;
    #endif
}

static inline void debug_capture_all_spi_mux_state(void) {
    for (uint8_t bus = 0; bus < 2; bus++) {
        if (spi_bus_in_use[bus]) {
            debug_capture_spi_mux_state(bus);
        }
    }
}

static inline void cs_set_value_checked(digitalio_digitalinout_obj_t *cs, bool value) {
    common_hal_digitalio_digitalinout_set_value(cs, value);
    debug_state.cs_set_calls++;
    debug_state.last_cs_bank = debug_cs_index(cs);
    debug_state.last_cs_target = value ? 1 : 0;
    debug_capture_cs_mux_state(cs);
    #if defined(PICO_RP2350)
    uint8_t gpio = cs->pin->number;
    bool out = rp_gpio_out(gpio);
    bool oe = rp_gpio_oe(gpio);
    bool in = rp_gpio_in(gpio);
    debug_state.last_cs_out = out ? 1 : 0;
    debug_state.last_cs_direction = oe ? 1 : 0;
    debug_state.last_cs_readback = in ? 1 : 0;
    if (out != value || !oe) {
        debug_state.cs_set_mismatch++;
    }
    #else
    debug_state.last_cs_out = value ? 1 : 0;
    debug_state.last_cs_direction = (uint8_t)common_hal_digitalio_digitalinout_get_direction(cs);
    bool readback = common_hal_digitalio_digitalinout_get_value(cs);
    debug_state.last_cs_readback = readback ? 1 : 0;
    if (readback != value) {
        debug_state.cs_set_mismatch++;
    }
    #endif
}

static bool is_all_zero_rx(const uint8_t *rx, size_t len) {
    if (rx == NULL || len == 0) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (rx[i] != 0x00) {
            return false;
        }
    }
    return true;
}

static void perform_one_time_miso_probe(uint8_t bus) {
    if (bus > 1 || (debug_state.miso_probe_valid_mask & (1u << bus)) != 0) {
        return;
    }

    const mcu_pin_obj_t *miso = spi_pin_config[bus][2];
    if (miso == NULL || !spi_bus_in_use[bus]) {
        return;
    }

    debug_state.miso_probe_triggered_mask |= (uint8_t)(1u << bus);

    for (uint8_t bank = 0; bank < bank_count; bank++) {
        if (spi_bus_by_bank[bank] == bus) {
            cs_set_value_checked(cs_for_bank(bank), true);
        }
    }

    if (!common_hal_busio_spi_deinited(spi_for_bus(bus))) {
        common_hal_busio_spi_deinit(spi_for_bus(bus));
    }

    #if defined(PICO_RP2350)
    uint8_t gpio = miso->number;
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_set_pulls(gpio, true, false);
    delay_microseconds(2);
    debug_state.miso_probe_pull_up[bus] = rp_gpio_in(gpio) ? 1 : 0;
    gpio_set_pulls(gpio, false, true);
    delay_microseconds(2);
    debug_state.miso_probe_pull_down[bus] = rp_gpio_in(gpio) ? 1 : 0;
    gpio_disable_pulls(gpio);
    #else
    debug_state.miso_probe_pull_up[bus] = 0xFF;
    debug_state.miso_probe_pull_down[bus] = 0xFF;
    #endif
    debug_state.miso_probe_valid_mask |= (uint8_t)(1u << bus);

    common_hal_busio_spi_construct(
        spi_for_bus(bus),
        spi_pin_config[bus][0],
        spi_pin_config[bus][1],
        spi_pin_config[bus][2],
        false);
    debug_capture_all_spi_mux_state();
    if (ensure_spi_lock_and_config(spi_for_bus(bus), spi_runtime_baudrate, spi_runtime_polarity, spi_runtime_phase)) {
        return;
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_INIT_SPI_LOCK);
}

static inline void debug_note_rx_for_probe(
    busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs, const uint8_t *rx, size_t len) {
    if (!is_all_zero_rx(rx, len)) {
        return;
    }
    uint8_t bus = debug_spi_index(spi);
    if (bus > 1) {
        return;
    }
    debug_state.all_zero_rx_count[bus]++;
    if ((debug_state.miso_probe_valid_mask & (1u << bus)) == 0) {
        debug_capture_cs_mux_state(cs);
        debug_capture_all_spi_mux_state();
        perform_one_time_miso_probe(bus);
    }
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

typedef struct {
    uint8_t *ptr;
    size_t len;
} u8_buffer_view_t;

typedef struct {
    uint16_t *ptr;
    size_t len;
} u16_buffer_view_t;

typedef struct {
    uint32_t *ptr;
    size_t len;
} u32_buffer_view_t;

// Write a 3-byte command frame: opcode + register + value.
static inline bool write_register3(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs, uint8_t op, uint8_t reg, uint8_t val) {
    uint8_t write_frame_3b[3] = {op, reg, val};
    cs_set_value_checked(cs, false);
    bool ok = common_hal_busio_spi_write(spi, write_frame_3b, sizeof(write_frame_3b));
    debug_note_xfer(spi, ok);
    cs_set_value_checked(cs, true);
    return ok;
}

// Issue a 2-byte command and read a 2-byte response with CS framing.
static inline bool transfer2(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs, const uint8_t *tx, uint8_t *rx) {
    cs_set_value_checked(cs, false);
    bool ok = common_hal_busio_spi_transfer(spi, tx, rx, 2);
    debug_note_xfer(spi, ok);
    if (ok) {
        debug_store_last_rx(rx, 2);
        debug_note_rx_for_probe(spi, cs, rx, 2);
    }
    cs_set_value_checked(cs, true);
    return ok;
}

// Issue a register-read command and read back one byte with CS framed once.
static inline bool read_register1(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs, uint8_t reg, uint8_t *out) {
    uint8_t register_read_frame_2b[2] = {TLA25x8_OP_REGISTER_READ, reg};
    cs_set_value_checked(cs, false);
    bool ok = common_hal_busio_spi_write(spi, register_read_frame_2b, sizeof(register_read_frame_2b));
    debug_note_xfer(spi, ok);
    if (ok) {
        ok = common_hal_busio_spi_read(spi, out, 1, 0x00);
        debug_note_xfer(spi, ok);
        if (ok) {
            debug_store_last_rx(out, 1);
        }
    }
    cs_set_value_checked(cs, true);
    return ok;
}

static inline bool pulse_device_reset(busio_spi_obj_t *spi, digitalio_digitalinout_obj_t *cs) {
    if (!write_register3(spi, cs, TLA25x8_OP_REGISTER_WRITE, TLA25x8_REG_GENERAL_CFG, TLA25x8_RST_MASK)) {
        // Write 0x01 to Register 0x01 (GENERAL_CFG)
        return false;
    }
    delay_microseconds(10000); // Datasheet says 5ms
    return true;
}

static inline bool bank_write_register(uint8_t bank, uint8_t reg, uint8_t value) {
    debug_set_bank(bank);
    return write_register3(
        spi_for_bank(bank),
        cs_for_bank(bank),
        TLA25x8_OP_REGISTER_WRITE,
        reg,
        value
        );
}

static inline bool bank_set_bits(uint8_t bank, uint8_t reg, uint8_t mask) {
    debug_set_bank(bank);
    return write_register3(
        spi_for_bank(bank),
        cs_for_bank(bank),
        TLA25x8_OP_BIT_SET,
        reg,
        mask
        );
}

static inline bool bank_clear_bits(uint8_t bank, uint8_t reg, uint8_t mask) {
    debug_set_bank(bank);
    return write_register3(
        spi_for_bank(bank),
        cs_for_bank(bank),
        TLA25x8_OP_BIT_CLEAR,
        reg,
        mask
        );
}

static inline bool bank_read_register(uint8_t bank, uint8_t reg, uint8_t *value) {
    debug_set_bank(bank);
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
    bool status_ok = bank_read_register(bank, TLA25x8_REG_SYSTEM_STATUS, &status);
    debug_store_bank_sanity_register(
        bank,
        debug_state.sanity_status_reg,
        &debug_state.sanity_status_valid_mask,
        status_ok,
        status);
    if (status_ok) {
        brown_out_by_bank[bank] = (status & TLA25x8_BOR_MASK) != 0;
        crcerr_fuse_by_bank[bank] = (status & TLA25x8_CRCERR_FUSE_MASK) != 0;
    } else {
        brown_out_by_bank[bank] = false;
        crcerr_fuse_by_bank[bank] = false;
    }

    uint8_t pin_cfg = 0;
    bool pin_cfg_ok = bank_read_register(bank, TLA25x8_REG_PIN_CFG, &pin_cfg);
    debug_store_bank_sanity_register(
        bank,
        debug_state.sanity_pin_cfg_reg,
        &debug_state.sanity_pin_cfg_valid_mask,
        pin_cfg_ok,
        pin_cfg);

    uint8_t general_cfg = 0;
    bool general_cfg_ok = bank_read_register(bank, TLA25x8_REG_GENERAL_CFG, &general_cfg);
    debug_store_bank_sanity_register(
        bank,
        debug_state.sanity_general_cfg_reg,
        &debug_state.sanity_general_cfg_valid_mask,
        general_cfg_ok,
        general_cfg);

    return status_ok;
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
    // Buffer lengths are expressed in bytes for both bytearray and array('H').
    size_t min_len_bytes = value_count * sizeof(uint16_t);
    if (bufinfo.len < min_len_bytes) {
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

    size_t min_len_bytes = readings_buffer_entry_count * sizeof(uint16_t);
    if (bufinfo.len < min_len_bytes) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("readings_buffer resized too small"));
    }

    view->is_bytes = readings_buffer_is_bytes;
    view->bytes = readings_buffer_is_bytes ? (uint8_t *)bufinfo.buf : NULL;
    view->u16 = readings_buffer_is_bytes ? NULL : (uint16_t *)bufinfo.buf;
}

static inline uint16_t read_sensor_value_at_index(const readings_buffer_view_t *view, size_t index) {
    if (view->is_bytes) {
        size_t offset = index * 2;
        return (uint16_t)view->bytes[offset] | ((uint16_t)view->bytes[offset + 1] << 8);
    }
    return view->u16[index];
}

static void acquire_u8_buffer_view(mp_obj_t buffer_obj, size_t min_len, u8_buffer_view_t *view) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_WRITE);
    uint8_t typecode = (uint8_t)(bufinfo.typecode & ~MP_OBJ_ARRAY_TYPECODE_FLAG_RW);
    bool is_bytes = typecode == BYTEARRAY_TYPECODE || typecode == 'B';
    if (!is_bytes) {
        mp_raise_ValueError(MP_ERROR_TEXT("expected 'B' buffer"));
    }
    view->ptr = (uint8_t *)bufinfo.buf;
    view->len = bufinfo.len;
    if (view->len < min_len) {
        mp_raise_ValueError(MP_ERROR_TEXT("state buffer too small"));
    }
}

static void acquire_u16_buffer_view(mp_obj_t buffer_obj, size_t min_len, u16_buffer_view_t *view) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_WRITE);
    uint8_t typecode = (uint8_t)(bufinfo.typecode & ~MP_OBJ_ARRAY_TYPECODE_FLAG_RW);
    if (typecode != 'H') {
        mp_raise_ValueError(MP_ERROR_TEXT("expected 'H' buffer"));
    }
    if ((bufinfo.len & 1u) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("unaligned 'H' buffer"));
    }
    view->ptr = (uint16_t *)bufinfo.buf;
    view->len = bufinfo.len / sizeof(uint16_t);
    if (view->len < min_len) {
        mp_raise_ValueError(MP_ERROR_TEXT("state buffer too small"));
    }
}

static void acquire_u32_buffer_view(mp_obj_t buffer_obj, size_t min_len, u32_buffer_view_t *view) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buffer_obj, &bufinfo, MP_BUFFER_WRITE);
    uint8_t typecode = (uint8_t)(bufinfo.typecode & ~MP_OBJ_ARRAY_TYPECODE_FLAG_RW);
    if (typecode != 'I') {
        mp_raise_ValueError(MP_ERROR_TEXT("expected 'I' buffer"));
    }
    if ((bufinfo.len & 3u) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("unaligned 'I' buffer"));
    }
    view->ptr = (uint32_t *)bufinfo.buf;
    view->len = bufinfo.len / sizeof(uint32_t);
    if (view->len < min_len) {
        mp_raise_ValueError(MP_ERROR_TEXT("state buffer too small"));
    }
}

static inline bool neighbor_is_active_for_guard(
    int16_t neighbor_idx,
    uint16_t active_sensors,
    const u8_buffer_view_t *sensor_disabled,
    const u16_buffer_view_t *sensor_min,
    const u16_buffer_view_t *sensor_max,
    const u16_buffer_view_t *latest_values,
    const u8_buffer_view_t *sensor_polarity,
    uint16_t min_event_range,
    uint8_t adjacent_guard_pct) {
    if (neighbor_idx < 0 || (uint16_t)neighbor_idx >= active_sensors) {
        return false;
    }
    size_t idx = (size_t)neighbor_idx;
    if (sensor_disabled->ptr[idx]) {
        return false;
    }

    uint16_t neighbor_min = sensor_min->ptr[idx];
    uint16_t neighbor_max = sensor_max->ptr[idx];
    uint16_t neighbor_val = latest_values->ptr[idx];
    uint16_t neighbor_low = neighbor_min < neighbor_val ? neighbor_min : neighbor_val;
    uint16_t neighbor_high = neighbor_max > neighbor_val ? neighbor_max : neighbor_val;
    uint16_t neighbor_rng = neighbor_high - neighbor_low;
    if (neighbor_rng < min_event_range) {
        return false;
    }

    if (sensor_polarity->ptr[idx] != 0) {
        uint16_t threshold = (uint16_t)(neighbor_high - ((uint32_t)neighbor_rng * adjacent_guard_pct) / 100u);
        return neighbor_val <= threshold;
    }
    uint16_t threshold = (uint16_t)(neighbor_low + ((uint32_t)neighbor_rng * adjacent_guard_pct) / 100u);
    return neighbor_val >= threshold;
}

static inline bool should_update_max_for_sensor(
    uint16_t idx,
    uint16_t active_sensors,
    const u8_buffer_view_t *sensor_disabled,
    const u16_buffer_view_t *sensor_min,
    const u16_buffer_view_t *sensor_max,
    const u16_buffer_view_t *latest_values,
    const u8_buffer_view_t *sensor_polarity,
    uint16_t min_event_range,
    uint8_t adjacent_guard_pct) {
    uint16_t rng = sensor_max->ptr[idx] - sensor_min->ptr[idx];
    if (rng < min_event_range) {
        return true;
    }
    if (neighbor_is_active_for_guard(
        (int16_t)idx - 1, active_sensors, sensor_disabled, sensor_min,
        sensor_max, latest_values, sensor_polarity, min_event_range, adjacent_guard_pct)) {
        return false;
    }
    if (neighbor_is_active_for_guard(
        (int16_t)idx + 1, active_sensors, sensor_disabled, sensor_min,
        sensor_max, latest_values, sensor_polarity, min_event_range, adjacent_guard_pct)) {
        return false;
    }
    return true;
}

static inline void write_sensor_value(
    readings_buffer_view_t *view, uint8_t bank, uint8_t slot, uint16_t value) {
    size_t index = sensor_value_index(bank, slot);
    debug_set_bank(bank);
    if (view->is_bytes) {
        size_t offset = index * 2;
        view->bytes[offset] = (uint8_t)(value & 0xFF);
        view->bytes[offset + 1] = (uint8_t)(value >> 8);
    } else {
        view->u16[index] = value;
    }
    debug_state.samples_written++;
}

static bool reset_all_banks_core(void) {
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        debug_set_bank(bank);
        cs_set_value_checked(cs_for_bank(bank), true);  // In case of a stale state
        if (!reset_bank_once(bank)) {
            debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_RESET_BANK);
            return false;
        }
    }
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        debug_set_bank(bank);
        if (!configure_bank_after_reset(bank)) {
            debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_CONFIGURE_BANK);
            return false;
        }
    }
    return true;
}

static bool refresh_all_bank_status_core(void) {
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        debug_set_bank(bank);
        if (!refresh_bank_status(bank)) {
            debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_REFRESH_STATUS);
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

static bool flush_spi_bus_mode0(busio_spi_obj_t *spi) {
    debug_state.last_bank = 0xFF;
    uint8_t bus = debug_spi_index(spi);
    if (bus <= 1) {
        debug_state.last_bus = bus;
    }
    if (!ensure_spi_lock_and_config(spi, 1000000, 0, 0)) {
        debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_SPI_FLUSH_CONFIG);
        return false;
    }
    delay_microseconds(10);
    bool ok = common_hal_busio_spi_write(spi, spi_flush_clock_bytes, sizeof(spi_flush_clock_bytes));
    debug_note_xfer(spi, ok);
    if (!ok) {
        debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_SPI_FLUSH_WRITE);
    }
    common_hal_busio_spi_unlock(spi);
    return ok;
}

static void assert_spi_pins_available(
    const mcu_pin_obj_t *const spi_pins[2][3],
    const bool bus_in_use[2]) {
    for (uint8_t bus = 0; bus < 2; bus++) {
        if (!bus_in_use[bus]) {
            continue;
        }
        for (uint8_t pin = 0; pin < 3; pin++) {
            if (!common_hal_mcu_pin_is_free(spi_pins[bus][pin])) {
                mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_PIN_CLAIM);
            }
        }
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
    for (uint8_t bus = 0; bus < 2; bus++) {
        if (!common_hal_busio_spi_deinited(spi_for_bus(bus))) {
            common_hal_busio_spi_deinit(spi_for_bus(bus));
        }
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
    memset(spi_pin_config, 0, sizeof(spi_pin_config));
    memset(spi_bus_in_use, 0, sizeof(spi_bus_in_use));
    memset(cs_pin_config, 0, sizeof(cs_pin_config));
    memset(spi_bus_by_bank, 0, sizeof(spi_bus_by_bank));
    spi_runtime_baudrate = 0;
    spi_runtime_polarity = 0;
    spi_runtime_phase = 0;
    debug_reset_state();
}

static void store_pin_configuration(
    const mcu_pin_obj_t *const spi_pins[2][3],
    const bool bus_in_use[2],
    const mcu_pin_obj_t *const cs_in[PHOTON_SENSORSCAN_MAX_BANKS], uint8_t bank_count_in) {
    for (uint8_t bus = 0; bus < 2; bus++) {
        spi_bus_in_use[bus] = bus_in_use[bus];
        for (uint8_t pin = 0; pin < 3; pin++) {
            spi_pin_config[bus][pin] = spi_pins[bus][pin];
        }
    }
    for (size_t i = 0; i < bank_count_in; i++) {
        cs_pin_config[i] = cs_in[i];
    }
}

static void construct_buses_and_cs(
    const mcu_pin_obj_t *const spi_pins[2][3],
    const bool bus_in_use[2],
    const mcu_pin_obj_t *const cs_in[PHOTON_SENSORSCAN_MAX_BANKS], uint8_t bank_count_in) {
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
        cs_set_value_checked(&cs_pins[i], true);
    }
    delay_microseconds(50000);
    for (uint8_t bus = 0; bus < 2; bus++) {
        if (!bus_in_use[bus]) {
            continue;
        }
        common_hal_busio_spi_construct(
            spi_for_bus(bus),
            spi_pins[bus][0],
            spi_pins[bus][1],
            spi_pins[bus][2],
            false);
        if (!flush_spi_bus_mode0(spi_for_bus(bus))) {
            mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_FLUSH);
        }
        debug_capture_spi_mux_state(bus);
    }
}

static inline uint16_t read_sensor_core(uint8_t bank, uint8_t slot) {
    // Sweep one emitter/sensor slot in a bank and return its ADC reading.
    debug_set_bank(bank);
    busio_spi_obj_t *spi = spi_for_bank(bank);
    digitalio_digitalinout_obj_t *cs = cs_for_bank(bank);
    uint8_t emitter_mask = emitter_mask_by_slot[slot];
    uint8_t adc_channel = adc_channel_by_slot[slot];
    uint8_t adc_result_2b[2];

    if (emitter_mask != 0) {
        debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_SET_EMITTER);
        if (!write_register3(spi, cs, TLA25x8_OP_BIT_SET, TLA25x8_REG_GPO_VALUE, emitter_mask)) {
            (void)refresh_bank_status(bank);
            mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
        }
    }

    delay_microseconds(settle_time_us);
    // In manual mode, channel switch is decoded on CS rising edge (cycle N), and
    // newly selected channel data are available in cycle N+2.
    // Frame 1: write MANUAL_CHID (select channel)
    // Frame 2: read/discard prior conversion
    // Frame 3: read selected channel conversion
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_SELECT_CHANNEL);
    if (!write_register3(spi, cs, TLA25x8_OP_REGISTER_WRITE, TLA25x8_REG_CHANNEL_SEL, adc_channel & 0x0F)) {
        if (emitter_mask != 0) {
            (void)write_register3(spi, cs, TLA25x8_OP_BIT_CLEAR, TLA25x8_REG_GPO_VALUE, emitter_mask);
        }
        (void)refresh_bank_status(bank);
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_PRIME_READ);
    if (!transfer2(spi, cs, adc_read_clock_2b, adc_result_2b)) {  // Discard first read to ensure proper channel
        if (emitter_mask != 0) {
            (void)write_register3(spi, cs, TLA25x8_OP_BIT_CLEAR, TLA25x8_REG_GPO_VALUE, emitter_mask);
        }
        (void)refresh_bank_status(bank);
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_DATA_READ);
    if (!transfer2(spi, cs, adc_read_clock_2b, adc_result_2b)) {
        if (emitter_mask != 0) {
            (void)write_register3(spi, cs, TLA25x8_OP_BIT_CLEAR, TLA25x8_REG_GPO_VALUE, emitter_mask);
        }
        (void)refresh_bank_status(bank);
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
    }

    uint16_t value = decode12(adc_result_2b);
    debug_state.drdy_count[bank]++;
    if (emitter_mask != 0) {
        debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_SCAN_CLEAR_EMITTER);
        if (!write_register3(spi, cs, TLA25x8_OP_BIT_CLEAR, TLA25x8_REG_GPO_VALUE, emitter_mask)) {
            (void)refresh_bank_status(bank);
            mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RUNTIME);
        }
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_NONE);
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

    debug_reset_state();
    spi_runtime_baudrate = baudrate;
    spi_runtime_polarity = polarity;
    spi_runtime_phase = phase;

    if (bank_count_in < 1 || bank_count_in > PHOTON_SENSORSCAN_MAX_BANKS) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid bank_count"));
    }
    if (slot_count_in > PHOTON_SENSORSCAN_MAX_SLOTS) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many slots"));
    }
    const mcu_pin_obj_t *spi_pins[2][3] = {
        {spi0_sck, spi0_mosi, spi0_miso},
        {spi1_sck, spi1_mosi, spi1_miso},
    };
    bool bus_in_use_next[2] = {false, false};
    for (size_t i = 0; i < bank_count_in; i++) {
        if (bank_spi_bus_in[i] >= 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("bank_spi_bus values must be 0 or 1"));
        }
        bus_in_use_next[bank_spi_bus_in[i]] = true;
    }
    for (uint8_t bus = 0; bus < 2; bus++) {
        bool has_sck = spi_pins[bus][0] != NULL;
        bool has_mosi = spi_pins[bus][1] != NULL;
        bool has_miso = spi_pins[bus][2] != NULL;
        bool has_any = has_sck || has_mosi || has_miso;
        bool has_all = has_sck && has_mosi && has_miso;
        if (has_any && !has_all) {
            mp_raise_ValueError(MP_ERROR_TEXT("SPI tuple must include sck/mosi/miso"));
        }
        if (bus_in_use_next[bus] && !has_all) {
            mp_raise_ValueError(bus == 0
                ? MP_ERROR_TEXT("spi0 required by bank_spi_bus")
                : MP_ERROR_TEXT("spi1 required by bank_spi_bus"));
        }
    }
    bool readings_buffer_is_bytes_in = false;
    size_t readings_value_count = 0;
    validate_readings_buffer(
        readings_buffer, bank_count_in, slot_count_in,
        &readings_buffer_is_bytes_in, &readings_value_count);

    bool need_construct = !is_initialized;
    if (is_initialized) {
        if (bank_count_in != bank_count) {
            mp_raise_ValueError(MP_ERROR_TEXT("bank_count cannot change"));
        }
        for (uint8_t bus = 0; bus < 2; bus++) {
            for (uint8_t pin = 0; pin < 3; pin++) {
                if (spi_pin_config[bus][pin] != spi_pins[bus][pin]) {
                    mp_raise_ValueError(MP_ERROR_TEXT("pins cannot change"));
                }
            }
            if (spi_bus_in_use[bus] != bus_in_use_next[bus]) {
                mp_raise_ValueError(MP_ERROR_TEXT("bank_spi_bus cannot change"));
            }
        }
        for (size_t i = 0; i < bank_count; i++) {
            if (cs_pin_config[i] != cs_in[i]) {
                mp_raise_ValueError(MP_ERROR_TEXT("pins cannot change"));
            }
            if (spi_bus_by_bank[i] != bank_spi_bus_in[i]) {
                mp_raise_ValueError(MP_ERROR_TEXT("bank_spi_bus cannot change"));
            }
            if (common_hal_digitalio_digitalinout_deinited(&cs_pins[i])) {
                need_construct = true;
            }
        }
        for (uint8_t bus = 0; bus < 2; bus++) {
            if (bus_in_use_next[bus] && common_hal_busio_spi_deinited(spi_for_bus(bus))) {
                need_construct = true;
            }
        }
    }

    if (need_construct) {
        if (is_initialized) {
            deinit_constructed_buses_and_cs();
        }
        is_initialized = false;

        assert_spi_pins_available(spi_pins, bus_in_use_next);
        assert_cs_pins_available(cs_in, bank_count_in);
        store_pin_configuration(spi_pins, bus_in_use_next, cs_in, bank_count_in);
        construct_buses_and_cs(spi_pins, bus_in_use_next, cs_in, bank_count_in);
    }

    for (uint8_t bus = 0; bus < 2; bus++) {
        if (!bus_in_use_next[bus]) {
            continue;
        }
        if (!ensure_spi_lock_and_config(spi_for_bus(bus), baudrate, polarity, phase)) {
            debug_state.last_bus = bus;
            debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_INIT_SPI_LOCK);
            mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_LOCK_CONFIG);
        }
    }

    bank_count = bank_count_in;
    slot_count = slot_count_in;
    settle_time_us = settle_us_in;
    for (uint8_t bus = 0; bus < 2; bus++) {
        spi_bus_in_use[bus] = bus_in_use_next[bus];
    }
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

    // Mark inactive before hardware reset/config so failures cannot leave a
    // partially updated configuration callable by refresh APIs.
    is_initialized = false;
    if (!reset_all_banks_core()) {
        deinit_constructed_buses_and_cs();
        clear_driver_state();
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET);
    }
    if (!refresh_all_bank_status_core()) {
        deinit_constructed_buses_and_cs();
        clear_driver_state();
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }

    MP_STATE_VM(photonsensorscan_readings_buffer_obj) = readings_buffer;
    readings_buffer_is_bytes = readings_buffer_is_bytes_in;
    readings_buffer_entry_count = readings_value_count;
    debug_capture_all_spi_mux_state();
    if (debug_state.last_cs_bank < bank_count) {
        debug_capture_cs_mux_state(cs_for_bank(debug_state.last_cs_bank));
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_NONE);
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

mp_obj_t common_hal_photonsensorscan_refresh_all_return(void) {
    ensure_initialized();
    if (slot_count == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("no slots configured"));
    }

    size_t total_values = (size_t)bank_count * (size_t)slot_count;
    mp_obj_t items[PHOTON_SENSORSCAN_MAX_BANKS * PHOTON_SENSORSCAN_MAX_SLOTS];
    size_t out_index = 0;
    for (uint8_t bank = 0; bank < bank_count; bank++) {
        for (uint8_t slot = 0; slot < slot_count; slot++) {
            uint16_t value = read_sensor_core(bank, slot);
            items[out_index++] = mp_obj_new_int_from_uint(value);
        }
    }
    return mp_obj_new_tuple(total_values, items);
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
    debug_set_bank(bank);
    if (!reset_bank_once(bank) ||
        !configure_bank_after_reset(bank)) {
        debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_RESET_BANK);
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET);
    }
    if (!refresh_bank_status(bank)) {
        debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_REFRESH_STATUS);
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_NONE);
}

void common_hal_photonsensorscan_reset_all_banks(void) {
    ensure_initialized();
    if (!reset_all_banks_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_RESET);
    }
    if (!refresh_all_bank_status_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_NONE);
}

void common_hal_photonsensorscan_refresh_status(void) {
    ensure_initialized();
    if (!refresh_all_bank_status_core()) {
        mp_raise_RuntimeError(PHOTON_SENSORSCAN_ERR_SPI_TRANSFER_STATUS);
    }
    debug_set_error(PHOTON_SENSORSCAN_DEBUG_ERR_NONE);
}

size_t common_hal_photonsensorscan_process_scan_events(
    mp_int_t active_sensors_in,
    mp_obj_t latest_values_obj,
    mp_obj_t sensor_disabled_obj,
    mp_obj_t sensor_min_obj,
    mp_obj_t sensor_max_obj,
    mp_obj_t sensor_polarity_obj,
    mp_obj_t sensor_strike_pct_obj,
    mp_obj_t sensor_on_obj,
    mp_obj_t sensor_active_obj,
    mp_obj_t sensor_strike_pending_obj,
    mp_obj_t sensor_strike_time_ms_obj,
    mp_obj_t sensor_release_pending_obj,
    mp_obj_t sensor_release_time_ms_obj,
    mp_int_t min_event_range_in,
    mp_int_t release_pct_in,
    mp_int_t activation_pct_in,
    mp_int_t adjacent_guard_pct_in,
    mp_int_t velocity_window_pct_in,
    mp_int_t strike_window_pct_in,
    mp_obj_t event_words_obj) {
    ensure_initialized();

    readings_buffer_view_t readings_view;
    acquire_readings_buffer_view(&readings_view);

    uint16_t active_sensors = (uint16_t)mp_arg_validate_int_range(
        active_sensors_in, 0, (mp_int_t)readings_buffer_entry_count, MP_QSTR_active_sensors);
    uint16_t min_event_range = (uint16_t)mp_arg_validate_int_range(
        min_event_range_in, 0, 4095, MP_QSTR_min_event_range);
    uint8_t release_pct = (uint8_t)mp_arg_validate_int_range(
        release_pct_in, 0, 100, MP_QSTR_release_pct);
    uint8_t activation_pct = (uint8_t)mp_arg_validate_int_range(
        activation_pct_in, 0, 100, MP_QSTR_activation_pct);
    uint8_t adjacent_guard_pct = (uint8_t)mp_arg_validate_int_range(
        adjacent_guard_pct_in, 0, 100, MP_QSTR_adjacent_guard_pct);
    uint8_t velocity_window_pct = (uint8_t)mp_arg_validate_int_range(
        velocity_window_pct_in, 0, 100, MP_QSTR_velocity_window_pct);
    uint8_t strike_window_pct = (uint8_t)mp_arg_validate_int_range(
        strike_window_pct_in, 0, 100, MP_QSTR_strike_window_pct);

    u16_buffer_view_t latest_values;
    u8_buffer_view_t sensor_disabled;
    u16_buffer_view_t sensor_min;
    u16_buffer_view_t sensor_max;
    u8_buffer_view_t sensor_polarity;
    u8_buffer_view_t sensor_strike_pct;
    u8_buffer_view_t sensor_on;
    u8_buffer_view_t sensor_active;
    u8_buffer_view_t sensor_strike_pending;
    u32_buffer_view_t sensor_strike_time_ms;
    u8_buffer_view_t sensor_release_pending;
    u32_buffer_view_t sensor_release_time_ms;
    u16_buffer_view_t event_words;

    acquire_u16_buffer_view(latest_values_obj, active_sensors, &latest_values);
    acquire_u8_buffer_view(sensor_disabled_obj, active_sensors, &sensor_disabled);
    acquire_u16_buffer_view(sensor_min_obj, active_sensors, &sensor_min);
    acquire_u16_buffer_view(sensor_max_obj, active_sensors, &sensor_max);
    acquire_u8_buffer_view(sensor_polarity_obj, active_sensors, &sensor_polarity);
    acquire_u8_buffer_view(sensor_strike_pct_obj, active_sensors, &sensor_strike_pct);
    acquire_u8_buffer_view(sensor_on_obj, active_sensors, &sensor_on);
    acquire_u8_buffer_view(sensor_active_obj, active_sensors, &sensor_active);
    acquire_u8_buffer_view(sensor_strike_pending_obj, active_sensors, &sensor_strike_pending);
    acquire_u32_buffer_view(sensor_strike_time_ms_obj, active_sensors, &sensor_strike_time_ms);
    acquire_u8_buffer_view(sensor_release_pending_obj, active_sensors, &sensor_release_pending);
    acquire_u32_buffer_view(sensor_release_time_ms_obj, active_sensors, &sensor_release_time_ms);
    acquire_u16_buffer_view(event_words_obj, 0, &event_words);

    size_t event_capacity = event_words.len / 3;
    size_t event_count = 0;
    uint32_t now_ms = supervisor_ticks_ms32();

    for (uint16_t sensor_idx = 0; sensor_idx < active_sensors; sensor_idx++) {
        if (sensor_disabled.ptr[sensor_idx]) {
            latest_values.ptr[sensor_idx] = 0;
            continue;
        }

        uint16_t value = read_sensor_value_at_index(&readings_view, sensor_idx);
        latest_values.ptr[sensor_idx] = value;
        if (value == 0) {
            continue;
        }

        if (value < sensor_min.ptr[sensor_idx]) {
            sensor_min.ptr[sensor_idx] = value;
        }
        if (value > sensor_max.ptr[sensor_idx] && should_update_max_for_sensor(
            sensor_idx, active_sensors, &sensor_disabled, &sensor_min, &sensor_max,
            &latest_values, &sensor_polarity, min_event_range, adjacent_guard_pct)) {
            sensor_max.ptr[sensor_idx] = value;
        }

        uint16_t min_v = sensor_min.ptr[sensor_idx];
        uint16_t max_v = sensor_max.ptr[sensor_idx];
        uint16_t rng = max_v - min_v;
        if (rng < min_event_range) {
            sensor_on.ptr[sensor_idx] = 0;
            sensor_active.ptr[sensor_idx] = 0;
            sensor_strike_pending.ptr[sensor_idx] = 0;
            sensor_strike_time_ms.ptr[sensor_idx] = 0;
            sensor_release_pending.ptr[sensor_idx] = 0;
            sensor_release_time_ms.ptr[sensor_idx] = 0;
            continue;
        }

        uint8_t strike_pct = sensor_strike_pct.ptr[sensor_idx];
        if (strike_pct > 100) {
            strike_pct = 100;
        }
        uint8_t velocity_start_pct = strike_pct > strike_window_pct ? strike_pct - strike_window_pct : 0;
        uint8_t release_velocity_pct = strike_pct + velocity_window_pct;
        if (release_velocity_pct > 100) {
            release_velocity_pct = 100;
        }
        uint8_t rel_pct = release_pct < strike_pct ? release_pct : strike_pct;

        bool reversed = sensor_polarity.ptr[sensor_idx] != 0;
        uint16_t activation_thr = 0;
        uint16_t strike_thr = 0;
        uint16_t velocity_start_thr = 0;
        uint16_t release_velocity_thr = 0;
        uint16_t release_thr = 0;
        bool is_active = false;
        if (!reversed) {
            activation_thr = (uint16_t)(min_v + ((uint32_t)rng * activation_pct) / 100u);
            strike_thr = (uint16_t)(min_v + ((uint32_t)rng * strike_pct) / 100u);
            velocity_start_thr = (uint16_t)(min_v + ((uint32_t)rng * velocity_start_pct) / 100u);
            release_velocity_thr = (uint16_t)(min_v + ((uint32_t)rng * release_velocity_pct) / 100u);
            release_thr = (uint16_t)(min_v + ((uint32_t)rng * rel_pct) / 100u);
            is_active = value >= activation_thr;
        } else {
            activation_thr = (uint16_t)(max_v - ((uint32_t)rng * activation_pct) / 100u);
            strike_thr = (uint16_t)(max_v - ((uint32_t)rng * strike_pct) / 100u);
            velocity_start_thr = (uint16_t)(max_v - ((uint32_t)rng * velocity_start_pct) / 100u);
            release_velocity_thr = (uint16_t)(max_v - ((uint32_t)rng * release_velocity_pct) / 100u);
            release_thr = (uint16_t)(max_v - ((uint32_t)rng * rel_pct) / 100u);
            is_active = value <= activation_thr;
        }
        sensor_active.ptr[sensor_idx] = is_active ? 1 : 0;

        if (!sensor_on.ptr[sensor_idx]) {
            if (!sensor_strike_pending.ptr[sensor_idx]) {
                if ((!reversed && value >= velocity_start_thr) ||
                    (reversed && value <= velocity_start_thr)) {
                    sensor_strike_pending.ptr[sensor_idx] = 1;
                    sensor_strike_time_ms.ptr[sensor_idx] = now_ms;
                }
            } else {
                bool crossed_strike = (!reversed && value >= strike_thr) ||
                    (reversed && value <= strike_thr);
                bool fell_back = (!reversed && value < velocity_start_thr) ||
                    (reversed && value > velocity_start_thr);
                if (crossed_strike) {
                    uint32_t dt = now_ms - sensor_strike_time_ms.ptr[sensor_idx];
                    if (dt > 0xFFFF) {
                        dt = 0xFFFF;
                    }
                    if (event_count < event_capacity) {
                        size_t base = event_count * 3;
                        event_words.ptr[base + 0] = sensor_idx;
                        event_words.ptr[base + 1] = 1;
                        event_words.ptr[base + 2] = (uint16_t)dt;
                        event_count++;
                    }
                    sensor_on.ptr[sensor_idx] = 1;
                    sensor_strike_pending.ptr[sensor_idx] = 0;
                    sensor_release_pending.ptr[sensor_idx] = 0;
                    sensor_release_time_ms.ptr[sensor_idx] = 0;
                } else if (fell_back) {
                    sensor_strike_pending.ptr[sensor_idx] = 0;
                }
            }
        } else {
            if (!sensor_release_pending.ptr[sensor_idx]) {
                if ((!reversed && value <= release_velocity_thr) ||
                    (reversed && value >= release_velocity_thr)) {
                    sensor_release_pending.ptr[sensor_idx] = 1;
                    sensor_release_time_ms.ptr[sensor_idx] = now_ms;
                }
            } else {
                bool crossed_release = (!reversed && value <= release_thr) ||
                    (reversed && value >= release_thr);
                bool fell_back = (!reversed && value > release_velocity_thr) ||
                    (reversed && value < release_velocity_thr);
                if (crossed_release) {
                    uint32_t dt = now_ms - sensor_release_time_ms.ptr[sensor_idx];
                    if (dt > 0xFFFF) {
                        dt = 0xFFFF;
                    }
                    if (event_count < event_capacity) {
                        size_t base = event_count * 3;
                        event_words.ptr[base + 0] = sensor_idx;
                        event_words.ptr[base + 1] = 0;
                        event_words.ptr[base + 2] = (uint16_t)dt;
                        event_count++;
                    }
                    sensor_on.ptr[sensor_idx] = 0;
                    sensor_strike_pending.ptr[sensor_idx] = 0;
                    sensor_release_pending.ptr[sensor_idx] = 0;
                    sensor_release_time_ms.ptr[sensor_idx] = 0;
                } else if (fell_back) {
                    sensor_release_pending.ptr[sensor_idx] = 0;
                }
            }
        }
    }

    return event_count;
}

uint8_t common_hal_photonsensorscan_slot_count(void) {
    return slot_count;
}

void common_hal_photonsensorscan_get_debug_state(photonsensorscan_debug_state_t *out_state) {
    if (out_state == NULL) {
        return;
    }
    memcpy(out_state, &debug_state, sizeof(*out_state));
}
