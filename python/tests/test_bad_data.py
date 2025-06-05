from ah_wrapper.ah_parser import parse_packet
from ah_wrapper.ppp_stuffing import PPPUnstuff

data = b"\x00~\x10\x91\x1f\x07\x00$)\xfc\xff:0\x03\x00\xf72\x0c\x00\xfa0\xec\xff\x12\xd6\xfa\xff\x850\x10\x92 \x18\xc3\xd0\x11\x05\x18\x0f\x9b\xd0\x1c\xe0\xd1E\x8c\xf6+\x9b\x00 <\x91!\xd3\x01\r\x9f\xd0N\xf6\xc1\x14\xb2\xd0\rc\xa4G\xdd\xc0\x13\x001~\x00"


def test_mock_bytearrays():
    unstuffer = PPPUnstuff()
    for b in data:
        unstuffed = unstuffer.unstuff_byte(b)
        if unstuffed:
            parsed = parse_packet(unstuffed)
    print("bar")
