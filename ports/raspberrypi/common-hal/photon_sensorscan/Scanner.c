// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "common-hal/photon_sensorscan/Scanner.h"

#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/microcontroller/__init__.h"

#include "py/runtime.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

#define NO_PIN 0xff

static const mcu_pin_obj_t *photon_sensorscan_adc_pin_for_channel(uint8_t channel) {
    switch (channel) {
        case 0:
            return &pin_GPIO26;
        case 1:
            return &pin_GPIO27;
        case 2:
            return &pin_GPIO28;
        case 3:
            return &pin_GPIO29;
        default:
            return NULL;
    }
}

static inline void photon_sensorscan_set_select(photon_sensorscan_scanner_obj_t *self, uint8_t channel) {
    gpio_put(self->sel0_pin, (channel & 0x01u) != 0);
    gpio_put(self->sel1_pin, (channel & 0x02u) != 0);
}

static uint16_t photon_sensorscan_read_adc(photon_sensorscan_scanner_obj_t *self) {
    uint16_t raw = 0;
    if (self->samples_per_channel <= 1) {
        raw = adc_read();
    } else {
        uint32_t accum = 0;
        for (uint8_t i = 0; i < self->samples_per_channel; i++) {
            accum += adc_read();
        }
        raw = (uint16_t)(accum / self->samples_per_channel);
    }
    return (uint16_t)((raw << 4) | (raw >> 8));
}

void common_hal_photon_sensorscan_construct(photon_sensorscan_scanner_obj_t *self,
    const mcu_pin_obj_t **enable_pins, size_t enable_pins_len,
    const mcu_pin_obj_t *sel0_pin, const mcu_pin_obj_t *sel1_pin,
    const mcu_pin_obj_t *adc_pin, int adc_channel,
    uint16_t settle_us, uint8_t samples_per_channel,
    uint8_t sensors_per_bank, uint16_t total_sensors) {

    self->enable_pin_numbers = NULL;
    self->bank_count = 0;
    self->sel0_pin = NO_PIN;
    self->sel1_pin = NO_PIN;
    self->adc_pin_number = NO_PIN;
    self->adc_channel = 0;
    self->update_id = 0;

    if (enable_pins_len == 0 || enable_pins_len > UINT8_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid enable_pins"));
    }
    if (sensors_per_bank == 0 || sensors_per_bank > 4) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid sensors_per_bank"));
    }
    if (samples_per_channel == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid samples_per_channel"));
    }
    size_t capacity = enable_pins_len * (size_t)sensors_per_bank;
    if (total_sensors == 0 || total_sensors > capacity) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid total_sensors"));
    }

    const mcu_pin_obj_t *adc_pin_for_claim = NULL;
    uint8_t resolved_adc_channel = 0;
    uint8_t resolved_adc_pin_number = NO_PIN;
    if (adc_pin != NULL) {
        if (adc_pin->number < ADC_BASE_PIN ||
            adc_pin->number > ADC_BASE_PIN + NUM_ADC_CHANNELS - 1) {
            raise_ValueError_invalid_pin();
        }
        resolved_adc_channel = (uint8_t)(adc_pin->number - ADC_BASE_PIN);
        adc_pin_for_claim = adc_pin;
        resolved_adc_pin_number = adc_pin->number;
    } else {
        if (adc_channel < 0 || adc_channel >= (int)NUM_ADC_CHANNELS) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid adc channel"));
        }
        resolved_adc_channel = (uint8_t)adc_channel;
        adc_pin_for_claim = photon_sensorscan_adc_pin_for_channel(resolved_adc_channel);
        if (adc_pin_for_claim != NULL) {
            resolved_adc_pin_number = adc_pin_for_claim->number;
        }
    }

    if (adc_pin_for_claim != NULL) {
        if (adc_pin_for_claim == sel0_pin || adc_pin_for_claim == sel1_pin) {
            mp_raise_ValueError(MP_ERROR_TEXT("pins must be distinct"));
        }
        for (size_t i = 0; i < enable_pins_len; i++) {
            if (adc_pin_for_claim == enable_pins[i]) {
                mp_raise_ValueError(MP_ERROR_TEXT("pins must be distinct"));
            }
        }
        assert_pin_free(adc_pin_for_claim);
    }

    self->enable_pin_numbers = (uint8_t *)m_malloc(enable_pins_len);
    if (self->enable_pin_numbers == NULL) {
        m_malloc_fail(enable_pins_len);
    }

    for (size_t i = 0; i < enable_pins_len; i++) {
        const mcu_pin_obj_t *pin = enable_pins[i];
        claim_pin(pin);
        self->enable_pin_numbers[i] = pin->number;
        gpio_init(pin->number);
        gpio_disable_pulls(pin->number);
        gpio_put(pin->number, false);
        gpio_set_dir(pin->number, GPIO_OUT);
    }

    claim_pin(sel0_pin);
    claim_pin(sel1_pin);
    gpio_init(sel0_pin->number);
    gpio_init(sel1_pin->number);
    gpio_disable_pulls(sel0_pin->number);
    gpio_disable_pulls(sel1_pin->number);
    gpio_put(sel0_pin->number, false);
    gpio_put(sel1_pin->number, false);
    gpio_set_dir(sel0_pin->number, GPIO_OUT);
    gpio_set_dir(sel1_pin->number, GPIO_OUT);

    if (adc_pin_for_claim != NULL) {
        claim_pin(adc_pin_for_claim);
    }
    adc_init();
    if (resolved_adc_pin_number != NO_PIN) {
        adc_gpio_init(resolved_adc_pin_number);
    }

    self->bank_count = (uint8_t)enable_pins_len;
    self->sel0_pin = sel0_pin->number;
    self->sel1_pin = sel1_pin->number;
    self->adc_pin_number = resolved_adc_pin_number;
    self->adc_channel = resolved_adc_channel;
    self->settle_us = settle_us;
    self->samples_per_channel = samples_per_channel;
    self->sensors_per_bank = sensors_per_bank;
    self->total_sensors = total_sensors;
}

bool common_hal_photon_sensorscan_deinited(photon_sensorscan_scanner_obj_t *self) {
    return self->enable_pin_numbers == NULL;
}

void common_hal_photon_sensorscan_deinit(photon_sensorscan_scanner_obj_t *self) {
    if (common_hal_photon_sensorscan_deinited(self)) {
        return;
    }

    for (size_t i = 0; i < self->bank_count; i++) {
        reset_pin_number(self->enable_pin_numbers[i]);
    }
    if (self->sel0_pin != NO_PIN) {
        reset_pin_number(self->sel0_pin);
        self->sel0_pin = NO_PIN;
    }
    if (self->sel1_pin != NO_PIN) {
        reset_pin_number(self->sel1_pin);
        self->sel1_pin = NO_PIN;
    }
    if (self->adc_pin_number != NO_PIN) {
        reset_pin_number(self->adc_pin_number);
        self->adc_pin_number = NO_PIN;
    }

    m_free(self->enable_pin_numbers);
    self->enable_pin_numbers = NULL;
    self->bank_count = 0;
    self->update_id = 0;
}

void common_hal_photon_sensorscan_scan_into(photon_sensorscan_scanner_obj_t *self, uint16_t *buffer) {
    adc_select_input(self->adc_channel);

    size_t idx = 0;
    const size_t total = self->total_sensors;

    for (uint8_t bank = 0; bank < self->bank_count && idx < total; bank++) {
        uint8_t bank_pin = self->enable_pin_numbers[bank];
        gpio_put(bank_pin, true);

        for (uint8_t channel = 0; channel < self->sensors_per_bank && idx < total; channel++) {
            photon_sensorscan_set_select(self, channel);
            if (self->settle_us > 0) {
                common_hal_mcu_delay_us(self->settle_us);
            }
            buffer[idx++] = photon_sensorscan_read_adc(self);
        }

        gpio_put(bank_pin, false);
    }

    self->update_id += 1;
}

uint32_t common_hal_photon_sensorscan_get_update_id(photon_sensorscan_scanner_obj_t *self) {
    return self->update_id;
}

uint16_t common_hal_photon_sensorscan_read_channel(photon_sensorscan_scanner_obj_t *self,
    uint8_t bank_index, uint8_t channel) {

    if (bank_index >= self->bank_count || channel >= self->sensors_per_bank) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid channel"));
    }

    adc_select_input(self->adc_channel);
    uint8_t bank_pin = self->enable_pin_numbers[bank_index];
    gpio_put(bank_pin, true);
    photon_sensorscan_set_select(self, channel);
    if (self->settle_us > 0) {
        common_hal_mcu_delay_us(self->settle_us);
    }
    uint16_t value = photon_sensorscan_read_adc(self);
    gpio_put(bank_pin, false);
    return value;
}
