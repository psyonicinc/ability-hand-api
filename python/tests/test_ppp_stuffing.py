import random

from ah_wrapper.ppp_stuffing import ESC_CHAR, FRAME_CHAR, PPPUnstuff, ppp_stuff
from copy import deepcopy


def test_stuff():
    buffer_size = 512
    large_array_solution = bytearray(
        [random.randint(0, 0x7C) for _ in range(buffer_size - 2)]
    )  # Large frame with data but no ESC_CHAR or FRAME_CHAR
    large_array = deepcopy(large_array_solution)
    large_array_solution.insert(0, FRAME_CHAR)
    large_array_solution.append(FRAME_CHAR)
    test_arrays = [
        bytearray([1, 2, ESC_CHAR, 3, 4, FRAME_CHAR]),
        bytearray([ESC_CHAR, FRAME_CHAR, ESC_CHAR, FRAME_CHAR]),
        large_array,
    ]
    result = [ppp_stuff(i) for i in test_arrays]
    solution = [
        bytearray(
            [
                FRAME_CHAR,
                1,
                2,
                ESC_CHAR,
                0x5D,
                3,
                4,
                ESC_CHAR,
                0x5E,
                FRAME_CHAR,
            ]
        ),
        bytearray(
            [
                FRAME_CHAR,
                ESC_CHAR,
                0x5D,
                ESC_CHAR,
                0x5E,
                ESC_CHAR,
                0x5D,
                ESC_CHAR,
                0x5E,
                FRAME_CHAR,
            ]
        ),
        large_array_solution,
    ]

    # Test Stuffing byte by byte
    for i in range(len(test_arrays)):
        for j in range(len(result)):
            assert result[i][j] == solution[i][j]


def test_unstuff():
    buffer_size = 512
    unstuff = PPPUnstuff(buffer_size=buffer_size)
    large_array_solution = bytearray(
        [random.randint(0, 0x7C) for _ in range(buffer_size - 2)]
    )  # Large frame with data but no ESC_CHAR or FRAME_CHAR
    large_array = deepcopy(large_array_solution)
    large_array_solution.insert(0, FRAME_CHAR)
    large_array_solution.append(FRAME_CHAR)
    test_arrays = [
        bytearray([1, 2, ESC_CHAR, 3, 4, FRAME_CHAR]),
        bytearray([ESC_CHAR, FRAME_CHAR, ESC_CHAR, FRAME_CHAR]),
        large_array,
    ]
    solution = [
        bytearray(
            [
                FRAME_CHAR,
                1,
                2,
                ESC_CHAR,
                0x5D,
                3,
                4,
                ESC_CHAR,
                0x5E,
                FRAME_CHAR,
            ]
        ),
        bytearray(
            [
                FRAME_CHAR,
                ESC_CHAR,
                0x5D,
                ESC_CHAR,
                0x5E,
                ESC_CHAR,
                0x5D,
                ESC_CHAR,
                0x5E,
                FRAME_CHAR,
            ]
        ),
        large_array_solution,
    ]

    # Test Un-stuffing via result
    for i in range(len(solution)):
        for b in solution[i]:
            frame = unstuff.unstuff_byte(b)
            if frame:
                assert frame == test_arrays[i]
