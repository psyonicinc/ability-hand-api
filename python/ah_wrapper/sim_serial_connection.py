import ah_wrapper.sim_serial as sim_serial
from ah_wrapper.serial_connection import SerialConnectionBase


class SimSerialConnection(SerialConnectionBase):
    def __init__(self, port: str, baud_rate: int, read_size: int = 512):
        super().__init__()

        # Variables
        self.read_size = read_size

        # Setup
        self._connect(port=port, baud_rate=baud_rate)

    def _connect(self, port: str, baud_rate: int):
        self._serial = sim_serial.Serial(
            port=port, baud_rate=baud_rate, read_size=self.read_size
        )

    def close(self):
        pass
