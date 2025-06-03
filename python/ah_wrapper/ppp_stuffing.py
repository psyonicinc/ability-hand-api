import logging
from typing import List
import os
import sys

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

FRAME_CHAR = 0x7E
ESC_CHAR = 0x7D
MASK_CHAR = 0x20


def ppp_stuff(
    array: bytearray | bytes | List[int], create_copy=False
) -> bytearray:
    """Stuffing involves adding a FRAME_CHAR 0x7E '~' to the begining and end of
    a frame and XOR'ing any bytes with MASK_CHAR 0x20 that equal the FRAME/ESC
    char.  This allows you to determine the beginning and end of a frame and not
    have FRAME_CHAR or ESC_CHAR that are actually in the data confuse the parsing
    of the frame"""
    if create_copy:  # I'm fine always modifying original array
        array = array.copy()

    # Find ESC and FRAME chars
    ind = [i for i, v in enumerate(array) if v == ESC_CHAR or v == FRAME_CHAR]

    for i in ind:  # Mask Chars
        array[i] = array[i] ^ MASK_CHAR

    # Insert ESC char in front of masked char reverse to prevent index mess up
    for i in sorted(ind, reverse=True):
        array.insert(i, ESC_CHAR)

    array.insert(0, FRAME_CHAR)  # Mark beginning of frame
    array.append(FRAME_CHAR)  # Mark end of the frame

    return array


class PPPUnstuff:
    def __init__(self, buffer_size=512):
        self.buffer_size = buffer_size
        self.buffer = bytearray(buffer_size)
        self.idx = 0
        self.unmask_next_char = False

    def add_to_buffer(self, byte: int):
        if self.idx >= self.buffer_size:
            if config.write_log:
                logging.warning("Exceeded maximum buffer size")
            self.idx = 0
        else:
            self.buffer[self.idx] = byte
            self.idx += 1

    def unstuff_byte(self, byte: int) -> bytearray | None:
        """Stateful byte parser for unstuffing PPP stuffed frames.  Unstuffing
        simply require you to remove the FRAME_CHAR 0x20 '~' byte from the end
        and beginning of the frame, it also requires removing any ESC_CHAR
        characters 0x7D (NOT ASCII) and XOR bytes that follow ESC_CHARS with
        MASK_CHAR 0x20.  This is required if the frame contains a FRAME_CHAR or
        ESC_CHAR not intended to be used for stuffing.

        General algo. is:
        - Read byte by byte and add to buffer
        - If ~ / FRAME_CHAR read, assume you are end of frame (even though you
        could be at the beginning)
            - If idx > 0 return true indicating a frame is found at
            self.buffer[:self.idx]
        - If ESC_CHAR is read, set unmask_next_char flag to true and continue to
        next byte and unmask it
        -
        """
        # If frame char, assume at the end of the frame, if no data, pass
        if byte == FRAME_CHAR:
            if self.idx:
                idx_copy = self.idx
                self.idx = 0
                return bytearray(self.buffer[0:idx_copy])  # No need for copy
            else:
                return None

        # Check for Escape Char and apply if true
        if byte == ESC_CHAR:
            # Next byte needs to be unmasked, next byte will never be FRAME_CHAR
            self.unmask_next_char = True
            return None
        if self.unmask_next_char:
            byte ^= MASK_CHAR
            self.unmask_next_char = False

        # Read Data
        self.add_to_buffer(byte)
        return None
