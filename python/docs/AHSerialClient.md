<a id="ah_serial_client"></a>

# ah\_serial\_client

<a id="ah_serial_client.AHSerialClient"></a>

## AHSerialClient Objects

```python
class AHSerialClient()
```

<a id="ah_serial_client.AHSerialClient.__init__"></a>

#### \_\_init\_\_

```python
def __init__(port: str = None,
             baud_rate: int = None,
             hand_address: int = 0x50,
             reply_mode: int = 0,
             read_size: int = 512,
             read_timeout: float = 0,
             write_timeout: float = 0.1,
             rs_485: bool = False,
             read_thread: bool = True,
             write_thread: bool = True,
             auto_start_threads: bool = True,
             simulated: str = "",
             rate_hz=500)
```

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

**Arguments**:

- `port` - i.e '/dev/ttyUSB0', auto determined if not passed
- `baud_rate` - Changes to signal per second, auto determined if not passed
- `hand_address` - Ability hand address identifier
- `reply_mode` - 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel.
- `read_size` - Size of serial read buffer
- `read_timeout` - Read timeout argument for serial.Serial class
- `write_timeout` - Write timeout argument for serial.Serial class
- `rs_485` - Set to True if using RS485
- `read_thread` - Creates a thread for parsing incoming bytes / status and reads at 1/(2*rate_hz) intervals
- `write_thread` - Creates a thread for writing self._command at 1/rate_hz intervals
- `auto_start_threads` - Automatically calls self.start_threads if True
- `simulated` - Argument for using various simulators in the future
- `rate_hz` - Rate argument that controls reads and write intervals.  If you wish to increase this you will need to increase the hands baud rate typically 921600 can handle a rate of 750 and 1000000 w/ RS485 can handle a rate of 1000

<a id="ah_serial_client.AHSerialClient.start_threads"></a>

#### start\_threads

```python
def start_threads()
```

Spins up threads, since public, ensures threads are not started twice

<a id="ah_serial_client.AHSerialClient.set_command"></a>

#### set\_command

```python
def set_command(command: bytearray | bytes | List[int]) -> None
```

Sets the private variable _command which is written to the hand using
the write thread or send_command function. Will not update hand targets
should be used more internally than externally

Args
    command: bytearray, bytes or list of ints with a command for AH

<a id="ah_serial_client.AHSerialClient.send_command"></a>

#### send\_command

```python
def send_command(command: bytearray | bytes | List[int] = None) -> None
```

Manually send command using the serial connection, if no command given
use the last used command. Useful if not using write thread but requires
you to send constantly to stay in API mode, does practically nothing if
write thread is running

**Arguments**:

- `command` - bytearray, bytes or list of ints with a command for AH

<a id="ah_serial_client.AHSerialClient.set_position"></a>

#### set\_position

```python
def set_position(positions: float | List[float], reply_mode=None, addr=None)
```

Set command to position target and update hand class position targets

If passing an array argument use the following index map:
[index, middle, ring, pinky, thumb flexor, thumb rotator]

**Arguments**:

- `positions` - (degrees - Thumb rotator[0,-100] other fingers [0,100]) either a single float or array for each finger all finger
- `reply_mode` - 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
- `addr` - Hand address, will use default if not passed

<a id="ah_serial_client.AHSerialClient.set_velocity"></a>

#### set\_velocity

```python
def set_velocity(velocities: float | List[float],
                 reply_mode=None,
                 addr=None) -> None
```

Set command to velocity target and update hand class position targets

If passing an array argument use the following index map:
[index, middle, ring, pinky, thumb flexor, thumb rotator]

**Arguments**:

- `velocities` - (degrees per second) either a single float or array for each finger
- `reply_mode` - 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
- `addr` - Hand address, will use default if not passed

<a id="ah_serial_client.AHSerialClient.set_duty"></a>

#### set\_duty

```python
def set_duty(duties: int | List[int], reply_mode=None, addr=None) -> None
```

Set command to duty target and update hand class duty targets

If passing an array argument use the following index map:
[index, middle, ring, pinky, thumb flexor, thumb rotator]

**Arguments**:

- `duties` - (duty cycle: [-100,100]) either a single int or array for each finger
- `reply_mode` - 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
- `addr` - Hand address, will use default if not passed

<a id="ah_serial_client.AHSerialClient.set_torque"></a>

#### set\_torque

```python
def set_torque(currents: float | List[float],
               reply_mode=None,
               addr=None) -> None
```

Set command to torque target and update hand class torque targets

If passing an array argument use the following index map:
[index, middle, ring, pinky, thumb flexor, thumb rotator]

**Arguments**:

- `currents` - [-1.0,1.0] either a single int or array for each finger
- `reply_mode` - 0: Pos. Cur. Touch 1: Pos. Vel. Touch 2: Pos. Cur. Vel., will use default if not passed
- `addr` - Hand address, will use default if not passed

<a id="ah_serial_client.AHSerialClient.set_grip"></a>

#### set\_grip

```python
def set_grip(grip: int, speed=0xFF)
```

Set grip command, see API for different grip commands

**Arguments**:

- `grip` - Grip command
- `speed` - When this value is between 1 and 254, the finger period will
  vary linearly between 2 seconds and .29 seconds. When this byte is 255,
  the finger period will be set to .2 seconds.

<a id="ah_serial_client.AHSerialClient.print_stats"></a>

#### print\_stats

```python
def print_stats() -> None
```

Prints stats regarding writes, reads and run time

<a id="ah_serial_client.AHSerialClient.close"></a>

#### close

```python
def close() -> None
```

Stop threads and close serial connection, will need to re-create
AHSerialClient instance to create a new connection

