// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Noah Jaffe for photon
//
// SPDX-License-Identifier: MIT

#define MICROPY_HW_BOARD_NAME "photon A003"
#define MICROPY_HW_MCU_NAME "rp2350a"


#define DEFAULT_I2C_BUS_SCL (&pin_GPIO7)
#define DEFAULT_I2C_BUS_SDA (&pin_GPIO6)

#define DEFAULT_SPI_BUS_SCK (&pin_GPIO10)
#define DEFAULT_SPI_BUS_MOSI (&pin_GPIO11)
#define DEFAULT_SPI_BUS_MISO (&pin_GPIO12)

#define DEFAULT_UART_BUS_RX (&pin_GPIO5)
#define DEFAULT_UART_BUS_TX (&pin_GPIO4)

// PSRAM chip select is wired to GPIO0 on this board.
#define CIRCUITPY_PSRAM_CHIP_SELECT (&pin_GPIO0)
