import logging
from typing import List
import threading
import time

import config
from ah_wrapper.serial_connection import SerialConnection
from ah_wrapper.hand import Hand
from ah_wrapper.ppp_stuffing import PPPUnstuff
from ah_wrapper.ah_parser import (
    parse_packet,
    Type1Packet,
    Type2Packet,
    Type3Packet,
)
from ah_wrapper.ah_api import (
    create_pos_msg,
    create_vel_msg,
    create_torque_msg,
    create_duty_msg,
    create_misc_msg,
    create_grip_msg,
)


class AHSerialClient:
    def __init__(
        self,
        port: str = None,
        baud_rate: int = None,
        hand_address: int = 0x50,
        reply_mode: int = 0,
        read_size: int = 512,
        read_timeout: float = 0,
        write_timeout: float = None,
        rs_485: bool = False,
        read_thread: bool = True,
        write_thread: bool = True,
        auto_start_threads: bool = True,
        simulated: bool = False,
        rate_hz=500,
    ):
        """
        Serial client and wrapper for ability hand containing functions for
        sending and receiving data to and from the ability hand. Uses the
        ability hand api to encode and parse byte array data sent to and from
        the ability hand.  Can automatically detect port and baud_rate but
        requires these arguments to be exclusively passed if using more than
        one hand.  If using two hands you will use two AHSerialClient instances
        and two serial adapters.  This code does not support two ability hands
        on the same bus using RS485.

        Typical use case is the private variable _command contains the desired
        command the user wishes to send to the ability hand and this _command is
        updated using any of the set functions.  If not using the built-in
        read and write threads, you will need to create your own.

        Args:
            port: i.e '/dev/ttyUSB0', auto determined if not passed
            baud_rate: Changes to signal per second, auto determined if not passed
            hand_address: Ability hand address identifier
            reply_mode: 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel.
            read_size: Size of serial read buffer
            read_timeout: Read timeout argument for serial.Serial class
            write_timeout: Write timeout argument for serial.Serial class
            rs_485:  Set to True if using RS485
            read_thread: Creates a thread for parsing incoming bytes / status and reads at 1/(2*rate_hz) intervals
            write_thread: Creates a thread for writing self._command at 1/rate_hz intervals
            auto_start_threads: Automatically calls self.start_threads if True
            simulated:  Will create a virtual serial connection
            rate_hz: Rate argument that controls reads and write intervals.  If you wish to increase this you will need to increase the hands baud rate typically 921600 can handle a rate of 750 and 1000000 w/ RS485 can handle a rate of 1000
        """
        if config.write_log:
            logging.basicConfig(
                filename="ability_hand.log",
                level=logging.INFO,
                format="%(asctime)s - %(levelname)s - %(message)s",
            )

        # Variables
        self.reply_mode = reply_mode
        self._read_size = read_size
        self._cmd_lock = threading.Lock()
        self._wait_time_s = 1 / rate_hz
        self._write_thread = None
        self._read_thread = None
        self._reading = False
        self._writing = False
        self.rate_hz = rate_hz
        if write_timeout is None:
            write_timeout = self._wait_time_s

        # Statistics Variables
        self.n_reads = 1
        self.start_time = time.time()
        self.end_time = None

        # Other classes
        self.hand = Hand(
            addr=hand_address
        )  # Hand class holds all hand properties
        self._unstuffer = PPPUnstuff(buffer_size=read_size)

        if simulated:
            from ah_simulators.sim_serial_connection import SimSerialConnection

            self._conn = SimSerialConnection(
                port=port, baud_rate=baud_rate, read_size=read_size
            )
        else:
            self._conn = SerialConnection(
                port=port,
                baud_rate=baud_rate,
                addr=hand_address,
                rs_485=rs_485,
                read_timeout=read_timeout,
                write_timeout=write_timeout,
            )

        # Initial hand position
        self.set_position(30)

        # Setup threads
        if read_thread:
            self._read_thread = threading.Thread(target=self._read_thread_loop)
        if write_thread:
            self._write_thread = threading.Thread(
                target=self._write_thread_loop
            )
        if auto_start_threads:
            self.start_threads()

    def start_threads(self):
        """Spins up threads, since public, ensures threads are not started twice"""
        if self._read_thread and not self._reading:
            self._reading = True
            if config.write_log:
                logging.info("Starting Read Thread")
            self._read_thread.start()
        elif self._reading:
            print("Read Thread already running")
        else:
            if config.write_log:
                logging.info("Read Thread not created")

        if self._write_thread and not self._writing:
            self._writing = True
            if config.write_log:
                logging.info("Starting Write Thread")
            self._write_thread.start()
        elif self._writing:
            print("Writing Thread already running")
        else:
            if config.write_log:
                logging.info("Write Thread not created")

    def _write_thread_loop(self):
        """Loop that constantly sends self._command"""
        while self._writing:
            with self._cmd_lock:
                self._conn.write(self._command)
            time.sleep(self._wait_time_s)

    def _read_thread_loop(self):
        """Thread which unstuffs frames, parses the packet of its respective type
        and updates Hand class, uses rate_hz to sleep in between reads to prevent
        CPU overload"""
        self.start_time = time.time()
        while self._reading:
            msg = self._conn.read(self._read_size)
            if msg:
                for b in msg:
                    frame = self._unstuffer.unstuff_byte(b)
                    if frame:
                        parsed = parse_packet(byte_array=frame)
                        if parsed is not None and parsed.valid:
                            self.n_reads += 1
                            try:
                                self.hand.update_hot_cold(parsed.hot_cold)

                                if type(parsed) == Type3Packet:
                                    self.hand._update_cur(
                                        positions=parsed.pos,
                                        velocity=parsed.vel,
                                        current=parsed.cur,
                                    )
                                elif type(parsed) == Type2Packet:
                                    self.hand._update_cur(
                                        positions=parsed.pos,
                                        velocity=parsed.vel,
                                        fsr=parsed.fsr,
                                    )
                                elif type(parsed) == Type1Packet:
                                    self.hand._update_cur(
                                        positions=parsed.pos,
                                        current=parsed.cur,
                                        fsr=parsed.fsr,
                                    )
                                else:
                                    if config.write_log:
                                        logging.warning(
                                            f"Invalid frame {frame}"
                                        )
                                        logging.warning(f"Invalid msg {msg}")
                            except:
                                if config.write_log:
                                    logging.warning(f"Bad frame {frame}")
                                    logging.warning(f"Bad msg {msg}")

            time.sleep(
                self._wait_time_s / 2
            )  # Sleeping on the job sometimes good
        self.end_time = time.time()
        if config.write_log:
            logging.info("Stopping Read Thread")

    def set_command(self, command: bytearray | bytes | List[int]) -> None:
        """
        Sets the private variable _command which is written to the hand using
        the write thread or send_command function. Will not update hand targets
        should be used more internally than externally

        Args
            command: bytearray, bytes or list of ints with a command for AH
        """
        with self._cmd_lock:
            self._command = command

    def send_command(
        self, command: None | bytearray | bytes | List[int] = None
    ) -> None:
        """
        Manually send command using the serial connection, if no command given
        use the last used command. Useful if not using write thread but requires
        you to send constantly to stay in API mode, does practically nothing if
        write thread is running

        Args:
            command: bytearray, bytes or list of ints with a command for AH
        """
        if command:
            self._conn.write(command)
        else:
            self._conn.write(self._command)

    def set_position(
        self, positions: float | List[float], reply_mode=None, addr=None
    ):
        """
        Set command to position target and update hand class position targets

        If passing an array argument use the following index map:
        [index, middle, ring, pinky, thumb flexor, thumb rotator]

        Args:
            positions: (degrees - Thumb rotator[0,-100] other fingers [0,100]) either a single float or array for each finger all finger
            reply_mode: 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
            addr: Hand address, will use default if not passed
        """
        if reply_mode is None:
            reply_mode = self.reply_mode
        if addr is None:
            addr = self.hand.addr
        self.set_command(
            create_pos_msg(
                reply_mode=reply_mode, positions=positions, addr=addr
            )
        )
        if type(positions) != list:
            positions = [positions] * 6
            positions[-1] *= -1
        self.hand.update_tar(positions=positions)

    def set_velocity(
        self, velocities: float | List[float], reply_mode=None, addr=None
    ) -> None:
        """
        Set command to velocity target and update hand class position targets

        If passing an array argument use the following index map:
        [index, middle, ring, pinky, thumb flexor, thumb rotator]

        Args:
            velocities: (degrees per second) either a single float or array for each finger
            reply_mode: 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
            addr: Hand address, will use default if not passed
        """
        if reply_mode is None:
            reply_mode = self.reply_mode
        if addr is None:
            addr = self.hand.addr
        self.set_command(
            create_vel_msg(
                reply_mode=reply_mode, velocities=velocities, addr=addr
            )
        )
        if type(velocities) != list:
            velocities = [velocities] * 6
            velocities[-1] *= -1
        self.hand.update_tar(velocities=velocities)

    def set_duty(
        self, duties: int | List[int], reply_mode=None, addr=None
    ) -> None:
        """
        Set command to duty target and update hand class duty targets

        If passing an array argument use the following index map:
        [index, middle, ring, pinky, thumb flexor, thumb rotator]

        Args:
            duties: (duty cycle: [-100,100]) either a single int or array for each finger
            reply_mode: 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
            addr: Hand address, will use default if not passed
        """
        if reply_mode is None:
            reply_mode = self.reply_mode
        if addr is None:
            addr = self.hand.addr
        self.set_command(
            create_duty_msg(reply_mode=reply_mode, duties=duties, addr=addr)
        )
        if type(duties) != list:
            duties = [duties] * 6
            duties[-1] *= -1
        self.hand.update_tar(duties=duties)

    def set_torque(
        self, currents: float | List[float], reply_mode=None, addr=None
    ) -> None:
        """
        Set command to torque target and update hand class torque targets

        If passing an array argument use the following index map:
        [index, middle, ring, pinky, thumb flexor, thumb rotator]

        Args:
            currents: [-1.0,1.0] either a single int or array for each finger
            reply_mode: 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
            addr: Hand address, will use default if not passed
        """
        if reply_mode is None:
            reply_mode = self.reply_mode
        if addr is None:
            addr = self.hand.addr
        self.set_command(
            create_torque_msg(
                reply_mode=reply_mode, currents=currents, addr=addr
            )
        )
        if type(currents) != list:
            currents = [currents] * 6
            currents[-1] *= -1
        self.hand.update_tar(currents=currents)

    def set_grip(self, grip: int, speed=0xFF, addr=None):
        """
        Set grip command, see API for different grip commands

        Args:
            grip: Grip command
            speed: When this value is between 1 and 254, the finger period will
            vary linearly between 2 seconds and .29 seconds. When this byte is 255,
            the finger period will be set to .2 seconds.
            addr: Hand address, will use default if not passed
        """
        if addr is None:
            addr = self.hand.addr
        self.set_command(create_grip_msg(cmd=grip, speed=speed, addr=addr))

    def print_stats(self) -> None:
        """Prints stats regarding writes, reads and run time"""
        if not self.end_time:
            end_time = time.time()
        else:
            end_time = self.end_time
        print(
            f"Rate: {((self._conn.n_writes-1) / (end_time - self.start_time)):.1f} \nWrites: {self._conn.n_writes-1} \nReads: {self.n_reads-1} \nPacket Loss: {(1 - ((self.n_reads-1) / (self._conn.n_writes-1))) * 100:.3f}%"
        )

    def close(self) -> None:
        """Stop threads and close serial connection, will need to re-create
        AHSerialClient instance to create a new connection"""
        self._reading = False
        self._writing = False
        if self._read_thread:
            self._read_thread.join()
        if self._write_thread:
            self._write_thread.join()
        self.print_stats()
        self._conn.close()
