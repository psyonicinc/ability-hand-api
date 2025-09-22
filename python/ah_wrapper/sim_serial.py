class Serial:
    """Simulated version of the python serial.Serial library with"""

    def __init__(self, port, baud_rate, read_size):
        self.buffer = []  # Unlimited buffer size allowed

    def read(self, read_size):
        return_buff = self.buffer[0:read_size]
        self.buffer = self.buffer[read_size:]
        return return_buff

    def write(self, msg):
        pass
