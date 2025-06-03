from abc import ABC, abstractmethod
import serial
import serial.rs485
import serial.tools.list_ports
import logging
import time
import platform
from typing import List
import threading
import os
import sys

from ah_wrapper.ah_api import create_misc_msg

CONFIG_PATH = os.path.join(os.getcwd(), "config.py")

# If config.py doesn't exist, create a basic one
if not os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, "w") as f:
        f.write(
            """# Auto-generated config file
write_log = True
velocity_warning = True
"""
        )

# Add current directory to sys.path so config.py can be imported
sys.path.insert(0, os.getcwd())

import config


class SerialConnectionBase(ABC):
    """Serial Connection Interface so we can simulate it in simulators and abstract
    away the try and excepts involved with reading and writing to a serial connection
    """

    def __init__(self):
        self._serial = None  # Each subclass will define
        self.rw_lock = (
            threading.Lock()
        )  # Avoid reading when writing visa versa
        self.n_writes = 1
        self.bauds = {
            0x10: 460800,
            0x11: 921600,
            0x12: 1000000,
            0x01: 1200,
            0x02: 2400,
            0x03: 4800,
            0x04: 9600,
            0x05: 14400,
            0x06: 19200,
            0x07: 28800,
            0x08: 31250,
            0x09: 38400,
            0x0A: 56000,
            0x0B: 57500,
            0x0C: 76800,
            0x0D: 115200,
            0x0E: 230400,
            0x0F: 250000,
        }

    @abstractmethod
    def _connect(self, port: str, baud_rate: int):
        pass

    def read(self, read_size: int = 512) -> bytes | None:
        try:
            with self.rw_lock:
                msg = self._serial.read(read_size)
            return msg
        except Exception as e:
            if config.write_log:
                logging.warning(e)

    def write(self, msg: bytes | bytearray | List[int]) -> None:
        try:
            with self.rw_lock:
                self._serial.write(msg)
                self.n_writes += 1
        except Exception as e:
            if config.write_log:
                logging.warning(e)

    def close(self) -> None:
        try:
            with self.rw_lock:
                self._serial.close()
        except Exception as e:
            if config.write_log:
                logging.warning(e)


class SerialConnection(SerialConnectionBase):
    def __init__(
        self,
        port,
        baud_rate,
        addr,
        rs_485,
        read_timeout,
        write_timeout,
    ):
        super().__init__()

        # Variables
        self.rs_485 = rs_485
        self.addr = addr

        # Setup
        connected, msg = self._connect(
            port=port,
            baud_rate=baud_rate,
            read_timeout=read_timeout,
            write_timeout=write_timeout,
        )
        if not connected:
            if config.write_log:
                logging.warning(f"Recieved {msg} when trying to connect")
            print("Could not connect to any hand, check logs for more info")
            exit(1)

    def _connect(
        self,
        read_timeout: float,
        write_timeout: float,
        port: str = None,
        baud_rate: int = None,
    ) -> (None | serial.Serial, bytearray):
        """Handles connections to various ports and baud_rates automatically return
        a tuple of (Serial Object, successful byte array response)"""

        test_msg = create_misc_msg(addr=self.addr, cmd=0xA2)
        if port and baud_rate:
            try:
                if self.rs_485:
                    self._serial = serial.rs485.RS485(
                        port=port,
                        baudrate=baud_rate,
                        timeout=read_timeout,
                        write_timeout=write_timeout,
                    )
                else:
                    self._serial = serial.Serial(
                        port=port,
                        baudrate=baud_rate,
                        timeout=read_timeout,
                        write_timeout=write_timeout,
                    )
                self._serial.write(test_msg)
                time.sleep(0.05)
                msg = self._serial.read(128)

                if len(msg) > 1:
                    return self._serial, msg
                else:
                    return None, msg

            except Exception as e:
                if config.write_log:
                    logging.warning(e)

        if not port:
            if platform.system() == "Linux":
                ports = [
                    p[0]
                    for p in serial.tools.list_ports.comports()
                    if "ttyUSB" in p[0] or "ttyACM" in p[0]
                ]
                for p in ports:
                    if not os.access(p, os.W_OK):
                        print(
                            f"Do not have permission to write to {p} please use:\nsudo chmod 666 {p}"
                        )
            elif platform.system() == "Darwin":
                ports = [
                    p[0]
                    for p in serial.tools.list_ports.comports()
                    if "usbserial" in p[0]
                ]
            elif platform.system() == "Windows":
                ports = [
                    p[0]
                    for p in serial.tools.list_ports.comports()
                    if "COM" in p[0]
                ]
            else:
                ports = [p[0] for p in serial.tools.list_ports.comports()]
        else:
            ports = [port]

        if not baud_rate:
            baud_rates = list(self.bauds.values())
        else:
            baud_rates = [baud_rate]

        for p in ports:
            for b in baud_rates:
                try:
                    if self.rs_485:
                        self._serial = serial.rs485.RS485(
                            port=p,
                            baudrate=b,
                            timeout=read_timeout,
                            write_timeout=write_timeout,
                        )
                    else:
                        self._serial = serial.Serial(
                            port=p,
                            baudrate=b,
                            timeout=read_timeout,
                            write_timeout=write_timeout,
                        )
                    self._serial.write(test_msg)
                    time.sleep(0.05)
                    msg = self._serial.read(128)
                    if len(msg) > 10:  # Valid response
                        print(f"Connected to: {p} with baudrate: {b}")
                        if config.write_log:
                            logging.info(
                                f"Connected to: {p} with baudrate: {b} confirmed with msg {msg}"
                            )
                        return self._serial, msg
                    else:
                        self._serial.close()
                except serial.SerialException as e:
                    if config.write_log:
                        logging.info(e)

        return None, None  # No connection found
