import random
from math import isclose

from ah_wrapper.ah_parser import parse_packet
from ah_wrapper.ppp_stuffing import PPPUnstuff
from ah_simulators.sim_functions import GeneratedPacket


def test_sm():
    parsed = False
    unstuffer = PPPUnstuff()
    pos = [random.random() * 90 for _ in range(6)]
    vel = [random.random() * 90 for _ in range(6)]
    packet = GeneratedPacket(reply_mode=2, mode=3, pos=pos, vel=vel)
    for b in packet.packet:
        unstuffed = unstuffer.unstuff_byte(b)
        if unstuffed:
            parsed = parse_packet(unstuffed)
    assert parsed
    for i in range(len(pos)):
        assert isclose(pos[i], packet.pos[i], abs_tol=0.15)
        assert isclose(vel[i], packet.vel[i], abs_tol=0.15)
