from ah_wrapper.hand import Hand
from ah_wrapper.observer import Observer

POS = [1, 2, 3, 4, 5, 6]
VEL = [0, 0, 0, 0, 0, 1]
CUR = [1.0, 1.0, 0, 0, 0, 0]
FSR = [0.5] * 30


class Foo(Observer):
    def update_pos(self, position):
        assert position == POS
        assert position != VEL
        assert position != CUR
        assert position != FSR

    def update_vel(self, velocity):
        assert velocity == VEL

    def update_cur(self, current):
        assert current == CUR

    def update_fsr(self, fsr):
        assert fsr == FSR


def test_observer():
    hand = Hand()
    foo = Foo()
    # Test that not having any observers works
    hand._update_cur(positions=POS)
    hand.add_observer(foo)
    # Test observers
    hand._update_cur(positions=POS)
    hand._update_cur(velocity=VEL)
    hand._update_cur(current=CUR)
    hand._update_cur(fsr=FSR)
