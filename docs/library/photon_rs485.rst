photon_rs485
=============

Photon-specific RS485 framing helpers with CRC32 and optional auto-reply.

This module is included only on boards that set ``CIRCUITPY_PHOTON_RS485 = 1``
in their ``mpconfigboard.mk`` (for example,
``ports/raspberrypi/boards/photon_rp2350a_main_board/mpconfigboard.mk`` and
``ports/raspberrypi/boards/photon_rp2350a_sensor_board/mpconfigboard.mk``).

Overview
--------

``photon_rs485`` wraps a UART and a DE (driver enable) pin to implement a simple
framed protocol with a fixed preamble and CRC32. It supports two operating
modes:

- **Auto-reply (C)** via :py:meth:`photon_rs485.RS485.process`, which handles
  ping/data/stats requests without Python loops.
- **Manual framing (Python)** via :py:meth:`photon_rs485.RS485.read_frames` and
  :py:meth:`photon_rs485.RS485.send_frame`.

The TX path uses DMA on RP2350 to reduce CPU overhead.

Quick start (auto-reply)
------------------------

.. code-block:: python

    import board
    import photon_rs485

    latest_values = [0] * 32
    scan_times = []

    bus = photon_rs485.RS485(
        tx=board.TX,
        rx=board.RX,
        de=board.DE,
        device_id=photon_rs485.DEVICE_MAIN,
        baudrate=2000000,
        tx_enable_delay_us=12,
    )

    while True:
        # update latest_values and scan_times in your own code
        bus.process(latest_values, scan_times)

Manual framing
--------------

.. code-block:: python

    for frame_type, target_id, payload, seq in bus.read_frames():
        if frame_type == photon_rs485.FRAME_TYPE_PING:
            bus.send_frame(photon_rs485.FRAME_TYPE_PONG, device_id, b"", seq)

API summary
-----------

.. class:: photon_rs485.RS485(tx, rx, de, *, baudrate=2000000, device_id=0, tx_enable_delay_us=12, rx_buffer_size=16384, max_payload=256)

   Construct an RS485 driver bound to a UART and DE pin. ``device_id=0``
   accepts all frames (host mode); other values only accept matching
   target IDs and broadcasts.

.. method:: photon_rs485.RS485.read_frames()

   Return a list of ``(frame_type, target_id, payload, seq)`` tuples.

.. method:: photon_rs485.RS485.send_frame(frame_type, target_id, payload, seq)

   Send a framed payload with CRC32.

.. method:: photon_rs485.RS485.process(latest_values, scan_times=None)

   Auto-reply to common request frames in C. ``latest_values`` may be a list of
   ints (packed as little-endian uint16) or a bytes-like object copied as-is.

Frame constants
---------------

``FRAME_PREAMBLE``, ``FRAME_HEADER_LEN``, ``FRAME_TRAILER_LEN``, plus:

- ``FRAME_TYPE_PING`` / ``FRAME_TYPE_PONG``
- ``FRAME_TYPE_DATA_REQ`` / ``FRAME_TYPE_DATA_RESP``
- ``FRAME_TYPE_STATS_REQ`` / ``FRAME_TYPE_STATS_RESP``
- ``DEVICE_MAIN`` / ``DEVICE_SECONDARY_1`` .. ``DEVICE_SECONDARY_5``
