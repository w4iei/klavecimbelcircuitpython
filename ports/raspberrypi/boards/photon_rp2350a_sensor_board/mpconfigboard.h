// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Noah Jaffe for photon
//
// SPDX-License-Identifier: MIT

#define MICROPY_HW_BOARD_NAME "photon A002"
#define MICROPY_HW_MCU_NAME "rp2350a"

// Just to set defaults, but any generic RP2350A UF2 should work for this sensor board
#define DEFAULT_UART_BUS_RX (&pin_GPIO23)
#define DEFAULT_UART_BUS_TX (&pin_GPIO22)

// PSRAM chip select is wired to GPIO0 on this board.

