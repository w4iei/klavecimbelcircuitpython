photon_sensorscan
=================

Photon-specific SPI sensor scanning helpers for TLA2518 banks.

This module is included only on boards that set ``CIRCUITPY_PHOTON_SENSORSCAN = 1``
in their ``mpconfigboard.mk``.

Overview
--------

``photon_sensorscan`` (also importable as ``photonsensorscan``) manages two SPI
buses and eight chip-select lines. Each bank maps to one TLA2518, and each
configured slot maps to an ADC channel plus optional emitter GPIO bit.

Call :func:`init` once to configure pin mappings and per-slot behavior, then use
:func:`scan_bank_into` or :func:`read_sensor`.

Basic usage
-----------

.. code-block:: python

    import array
    import board
    import photon_sensorscan

    photon_sensorscan.init(
        spi0=(board.GP2, board.GP3, board.GP4),
        spi1=(board.GP6, board.GP7, board.GP8),
        cs=(board.GP9, board.GP10, board.GP11, board.GP12,
            board.GP13, board.GP14, board.GP15, board.GP16),
        adc_channels=(0, 1, 2, 3),
        emitter_bits=(-1, -1, 0, 1),
        settle_us=50,
        baudrate=15_000_000,
        polarity=0,
        phase=0,
    )

    buf = array.array("H", [0] * 4)
    photon_sensorscan.scan_bank_into(0, buf)
    one_value = photon_sensorscan.read_sensor(0, 2)

API summary
-----------

.. function:: photon_sensorscan.init(*, spi0, spi1, cs, adc_channels, emitter_bits, settle_us=50, baudrate=15000000, polarity=0, phase=0)

   Initialize module-owned SPI interfaces, chip-select outputs, and slot mapping.
   ``spi0`` and ``spi1`` are ``(sck, mosi, miso)`` tuples. ``cs`` must contain
   exactly 8 chip-select pins. ``adc_channels`` and ``emitter_bits`` must have
   the same length (up to 8 slots).

.. function:: photon_sensorscan.scan_bank_into(bank, out)

   Scan all configured slots for ``bank`` into writable ``out``.

.. function:: photon_sensorscan.read_sensor(bank, slot)

   Read a single configured slot from ``bank`` and return the 12-bit ADC value.
