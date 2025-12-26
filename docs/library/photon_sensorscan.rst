photon_sensorscan
=================

Photon-specific real-time sensor scanning helpers.

This module is included only on boards that set ``CIRCUITPY_PHOTON_SENSORSCAN = 1``
in their ``mpconfigboard.mk`` (for example,
``ports/raspberrypi/boards/photon_rp2350a_main_board/mpconfigboard.mk`` and
``ports/raspberrypi/boards/photon_rp2350a_sensor_board/mpconfigboard.mk``).

Overview
--------

``photon_sensorscan`` owns the GPIO bank enable pins, mux select pins, and a
single ADC input to scan a sensor array with deterministic timing. Scans are
bank-major and channel-minor, starting at channel 0 for each bank. The driver
writes exactly ``total_sensors`` samples per scan and increments ``update_id``
each time. With two select pins, up to four channels per bank are addressable.

Basic usage
-----------

.. code-block:: python

    import array
    import photon_sensorscan
    import board

    scanner = photon_sensorscan.Scanner(
        enable_pins=(board.D2, board.D3),
        sel0_pin=board.D4,
        sel1_pin=board.D5,
        adc_pin=board.A0,
        settle_us=5,
        sensors_per_bank=4,
        total_sensors=6,
    )

    buf = array.array("H", [0] * 6)
    scanner.scan_into(buf)

API summary
-----------

.. class:: photon_sensorscan.Scanner(enable_pins, sel0_pin, sel1_pin, adc_pin, *, settle_us, samples_per_channel=1, sensors_per_bank, total_sensors)

   Create a scanner bound to GPIO bank enables, mux selects, and one ADC input.
   ``adc_pin`` can be an ADC-capable pin or an ADC channel index. When
   ``samples_per_channel`` is greater than 1, samples are averaged per channel.

.. method:: photon_sensorscan.Scanner.scan_into(buffer)

   Scan the full sensor array into ``buffer``. ``buffer`` must hold
   ``total_sensors`` 16-bit values (for example ``array('H')`` or ``bytearray``).

.. method:: photon_sensorscan.Scanner.read_channel(bank, channel)

   Read a single bank/channel (debug helper).

.. attribute:: photon_sensorscan.Scanner.update_id

   Monotonic counter incremented after each full scan. (read-only)
