import struct
import time
from collections import namedtuple
from numbers import Real
from typing import Iterator

import serial

Datapoint = namedtuple('Datapoint',
                       ('update_time', 'delta_t', 'valve_motion_state', 'water_level', 'water_level_reading',
                        'i2', 'i', 'error', 'd', 'd2', 'activation', 'to_move', 'est_position'))


class ControllerReader:
    class _ReaderContext:
        _decoder = struct.Struct('<LfhB9f4x')
        _data = b''

        def __init__(self, ser: serial.Serial):
            self._ser = ser
            self._ser.read_until(b'\xDE\xAD\xBE\xEF')

        def read_single(self) -> Datapoint:
            self._data += self._ser.read_until(b'\xDE\xAD\xBE\xEF')
            if len(self._data) != self._decoder.size:
                self._data = self._ser.read_until(b'\xDE\xAD\xBE\xEF')
            try:
                unpacked_data = self._decoder.unpack(self._data)
            except struct.error:
                #print(len(self._data))
                #print(self._data)
                raise
            self._data = b''
            return Datapoint(*unpacked_data)

        def iter_readings(self) -> Iterator[Datapoint]:
            while self._ser.is_open:
                try:
                    yield self.read_single()
                except struct.error:
                    return

        def read_for(self, seconds: Real) -> Iterator[Datapoint]:
            it = self.iter_readings()
            time_remaining = seconds
            old_timeout = self._ser.timeout
            while time_remaining > 0:
                self._ser.timeout = time_remaining
                last_time = time.time()
                try:
                    yield next(it)
                except StopIteration:
                    self._ser.timeout = old_timeout
                    return
                time_remaining -= time.time() - last_time

    def __init__(self, port: str = "COM3"):
        self._ser = serial.Serial(baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, timeout=2)
        self._ser.port = port

    def __enter__(self) -> _ReaderContext:
        self._ser.__enter__()
        return self._ReaderContext(self._ser)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._ser.__exit__(exc_type, exc_val, exc_tb)
