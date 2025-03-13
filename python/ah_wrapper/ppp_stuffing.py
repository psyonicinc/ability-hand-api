from enum import Enum
import logging
from typing import List

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


class PPPState(Enum):
    START_FRAME = 0
    DATA = 1
    END_FRAME = 2


class PPPUnstuff:
    def __init__(self, buffer_size=512):
        self.state = PPPState.START_FRAME
        self.buffer_size = buffer_size
        self.buffer = bytearray(buffer_size)
        self.idx = 0
        self.unmask_next_char = False

    def reset_state(self):
        self.state = PPPState.START_FRAME
        self.idx = 0

    def add_to_buffer(self, byte: int):
        if self.idx >= self.buffer_size:
            if config.write_log:
                logging.warning("Exceeded maximum buffer size")
            self.reset_state()
        else:
            self.buffer[self.idx] = byte
            self.idx += 1

    def unstuff_byte(self, byte: int) -> None | bytearray:
        """Stateful byte parser for unstuffing PPP stuffed frames.  Unstuffing
        simply require you to remove the FRAME_CHAR 0x20 '~' byte from the end
        and beginning of the frame, it also requires removing any ESC_CHAR
        characters 0x7D (NOT ASCII) and XOR bytes that follow ESC_CHARS with
        MASK_CHAR 0x20.  This is required if the frame contains a FRAME_CHAR or
        ESC_CHAR not intended to be used for stuffing. Really only needs to be a
        one state state machine, read data, or don't, but state machine helps
        with readability and understanding the if statements"""
        if byte != FRAME_CHAR:
            # If we see a non frame char and not in a reading data state, skip
            if self.state != PPPState.DATA:
                return None
        else:
            if self.idx > 0:
                # We are at the end of a frame because we have data
                self.state = PPPState.END_FRAME  # Just here for readability
                idx_copy = self.idx  # Annoying...
                self.reset_state()  # Resets idx and sets state to start
                return bytearray(
                    self.buffer[0:idx_copy]
                )  # Creates a copy because it would definitely suck if we read a byte before processing the returned byte array
            else:
                # We are at the beginning of a frame
                self.reset_state()
                self.state = PPPState.DATA
                return None
        if (
            byte == ESC_CHAR
        ):  # Next byte needs to be unmasked, next byte will never be FRAME_CHAR
            self.unmask_next_char = True
            return None
        if self.unmask_next_char:
            byte ^= MASK_CHAR
            self.unmask_next_char = False

        # Data read state
        if self.state == PPPState.DATA:
            self.add_to_buffer(byte)
            return None
