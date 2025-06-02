from threading import Lock
from typing import List
import logging

import config


class Hand:
    def __init__(self, addr: int = 0x50, fsr_offset: bool = True):
        """Hand class used to represent the state of the real or virtual hand.

        Args:
            addr: address of hand
            fsr_offset: If true will subtract the initial fsr readings from future readings
        """
        self.addr = addr
        self._hot_cold = None  # Hot cold status
        self._cur_pos = None  # Current pos
        self._tar_pos = None  # Target pos
        self._cur_vel = None  # Current vel
        self._tar_vel = None
        self._cur_cur = None  # Current current :)
        self._tar_cur = None
        self._tar_duty = None  # Target duty/voltage we don't read duty/voltage
        self._fsr = None  # Current FSR readings
        self._fsr_offset = [0] * 30
        self._first_fsr = fsr_offset

        self._val_lock = Lock()
        self._tar_lock = Lock()

    def _update_cur(
        self,
        positions: List[float] = None,
        velocity: List[float] = None,
        current: List[float] = None,
        fsr: List[float] = None,
    ):
        """Safely updates most recent readings from hand based on feedback"""
        with self._val_lock:
            if positions:
                self._cur_pos = positions
            if velocity:
                self._cur_vel = velocity
            if current:
                self._cur_cur = current
            if fsr:
                if self._first_fsr and 0 not in fsr:
                    self._fsr_offset = [-i for i in fsr]
                    if config.write_log:
                        logging.info(
                            f"Applied FSR offset of: {self._fsr_offset}"
                        )
                    self._first_fsr = False
                self._fsr = [
                    fsr[i] + self._fsr_offset[i] for i in range(len(fsr))
                ]

    def update_tar(
        self,
        positions: List[float] = None,
        velocities: List[float] = None,
        currents: List[float] = None,
        duties: List[float] = None,
    ):
        """Safely updates the most recent target position, velocity, current or
        duty.  Useful for in the loop feedback algo. such as PID controllers
        NOTE: These targets do not affect the control of the hand at all, that
        is done by the AHSerialClient class, anytime that class issues a set
        command it will update these targets with what it is sending.  At this
        time, to avoid confusion, there can only be one target at a time."""
        with self._tar_lock:
            if positions:
                self._tar_pos = positions
                self._tar_vel = None
                self._tar_cur = None
                self._tar_duty = None
            elif velocities:
                self._tar_vel = velocities
                self._tar_pos = None
                self._tar_cur = None
                self._tar_duty = None
            elif currents:
                self._tar_cur = currents
                self._tar_vel = None
                self._tar_pos = None
                self._tar_duty = None
            elif duties:
                self._tar_duty = duties
                self._tar_cur = None
                self._tar_vel = None
                self._tar_pos = None

    def update_hot_cold(self, hot_cold_status: int) -> None:
        with self._val_lock:
            self._hot_cold = hot_cold_status

    def get_hot_cold(self) -> int:
        with self._val_lock:
            return self._hot_cold

    def get_current(self) -> None | List[float]:
        """Returns most recent finger current feedback in amps or None"""
        with self._val_lock:
            return self._cur_cur

    def get_position(self) -> None | List[float]:
        """Returns most recent finger position feedback in degrees or None"""
        with self._val_lock:
            return self._cur_pos

    def get_velocity(self) -> None | List[float]:
        """Returns most recent finger velocity feedback in degrees per second or None"""
        with self._val_lock:
            return self._cur_vel

    def get_fsr(self) -> None | List[float]:
        """Returns most recent FSR touch sensor values or None"""
        with self._val_lock:
            return self._fsr

    def get_tar_position(self) -> None | List[float]:
        """Returns hand target positions / last position command or None"""
        with self._tar_lock:
            return self._tar_pos

    def get_tar_velocity(self) -> None | List[float]:
        """Returns hand target velocities / last velocity command or None"""
        with self._tar_lock:
            return self._tar_vel

    def get_tar_current(self) -> None | List[float]:
        """Returns hand target currents / last current command or None"""
        with self._tar_lock:
            return self._tar_cur

    def get_tar_duty(self) -> None | List[float]:
        """Returns hand target duty cycle / last duty command or None"""
        with self._tar_lock:
            return self._tar_duty

    def __repr__(self):
        string = ""
        if self._cur_pos is not None:
            string = f"Positions: "
            for i in range(len(self._cur_pos)):
                string += f"{self._cur_pos[i]:.2f} "

        if self._cur_cur is not None:
            string += "\nCurrents: "
            for i in range(len(self._cur_cur)):
                string += f"{self._cur_cur[i]:.2f} "

        if self._cur_vel is not None:
            string += "\nVelocity: "
            for i in range(len(self._cur_vel)):
                string += f"{self._cur_vel[i]:.2f} "

        if self._fsr is not None:
            keys = ("Index", "Middle", "Ring", "Pinky", "Thumb")
            for i in range(5):
                string += f"\nFSR {keys[i]}: "
                for j in range(6):
                    string += f"{self._fsr[j+i*6]:.2f} "

        return string
