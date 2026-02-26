// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Noah Jaffe for klavecimbel.com
//
// SPDX-License-Identifier: MIT

#define MICROPY_HW_BOARD_NAME "klavecimbel.com mcu dev board"
#define MICROPY_HW_MCU_NAME "rp2350b"


#define DEFAULT_I2C_BUS_SCL (&pin_GPIO7)
#define DEFAULT_I2C_BUS_SDA (&pin_GPIO6)

#define DEFAULT_SPI_BUS_SCK (&pin_GPIO2)
#define DEFAULT_SPI_BUS_MOSI (&pin_GPIO3)
#define DEFAULT_SPI_BUS_MISO (&pin_GPIO4)

#define DEFAULT_UART_BUS_RX (&pin_GPIO1)
#define DEFAULT_UART_BUS_TX (&pin_GPIO0)

