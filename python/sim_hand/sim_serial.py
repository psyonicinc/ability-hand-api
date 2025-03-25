class Serial:
    """Simulated version of the python serial.Serial library with"""

    def __init__(self, port: str, baud_rate: int, read_size: int):
        self.buffer = []  # Unlimited buffer size allowed
        self.read_size = read_size
        pass

    def read(self):
        return_buff = self.buffer[0 : self.read_size]
        self.buffer = self.buffer[self.read_size :]
        return return_buff
