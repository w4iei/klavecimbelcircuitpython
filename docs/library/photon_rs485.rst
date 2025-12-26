photon_rs485
=============

Photon-specific RS485 framing helpers with CRC32.

This module is included only on boards that set ``CIRCUITPY_PHOTON_RS485 = 1``
in their ``mpconfigboard.mk`` (for example,
``ports/raspberrypi/boards/photon_rp2350a_main_board/mpconfigboard.mk`` and
``ports/raspberrypi/boards/photon_rp2350a_sensor_board/mpconfigboard.mk``).

Overview
--------

``photon_rs485`` wraps a UART and a DE (driver enable) pin to implement a simple
framed protocol with a fixed preamble and CRC32. Use
:py:meth:`photon_rs485.RS485.read_frames` and
:py:meth:`photon_rs485.RS485.send_frame` to manage framing in Python, or set a
one or more auto-reply handlers for request/response polling.

The TX path uses DMA on RP2350 to reduce CPU overhead.

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

.. method:: photon_rs485.RS485.set_auto_reply(request_type, response_type, payload)

   Configure auto-replies for ``read_frames``. Pass ``None`` to disable and
   clear any existing entries. Use a ``bytearray`` to update contents in place.

.. method:: photon_rs485.RS485.add_auto_reply(request_type, response_type, payload)

   Register an additional auto-reply handler for ``read_frames``.

Frame constants
---------------

``FRAME_PREAMBLE``, ``FRAME_HEADER_LEN``, ``FRAME_TRAILER_LEN``, plus:

- ``FRAME_TYPE_PING`` / ``FRAME_TYPE_PONG``
- ``FRAME_TYPE_DATA_REQ`` / ``FRAME_TYPE_DATA_RESP``
- ``FRAME_TYPE_STATS_REQ`` / ``FRAME_TYPE_STATS_RESP``
- ``FRAME_TYPE_EVENT`` / ``FRAME_TYPE_EVENT_ACK``
- ``FRAME_TYPE_CFG_SET`` / ``FRAME_TYPE_CFG_ACK``
- ``FRAME_TYPE_MINMAX_REQ`` / ``FRAME_TYPE_MINMAX_RESP``
- ``FRAME_TYPE_TRACE_REQ`` / ``FRAME_TYPE_TRACE_RESP``
- ``DEVICE_MAIN`` / ``DEVICE_SECONDARY_1`` .. ``DEVICE_SECONDARY_5``
