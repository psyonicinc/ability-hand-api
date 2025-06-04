<a id="hand"></a>

# hand

<a id="hand.Hand"></a>

## Hand Objects

```python
class Hand(Observable)
```

<a id="hand.Hand.__init__"></a>

#### \_\_init\_\_

```python
def __init__(addr: int = 0x50, fsr_offset: bool = True)
```

Hand class used to represent the state of the real or virtual hand.

**Arguments**:

- `addr` - address of hand
- `fsr_offset` - If true will subtract the initial fsr readings from future readings

<a id="hand.Hand.update_tar"></a>

#### update\_tar

```python
def update_tar(positions: List[float] = None,
               velocities: List[float] = None,
               currents: List[float] = None,
               duties: List[float] = None)
```

Safely updates the most recent target position, velocity, current or
duty.  Useful for in the loop feedback algo. such as PID controllers
NOTE: These targets do not affect the control of the hand at all, that
is done by the AHSerialClient class, anytime that class issues a set
command it will update these targets with what it is sending.  At this
time, to avoid confusion, there can only be one target at a time.

<a id="hand.Hand.get_current"></a>

#### get\_current

```python
def get_current() -> None | List[float]
```

Returns most recent finger current feedback in amps or None

<a id="hand.Hand.get_position"></a>

#### get\_position

```python
def get_position() -> None | List[float]
```

Returns most recent finger position feedback in degrees or None

<a id="hand.Hand.get_velocity"></a>

#### get\_velocity

```python
def get_velocity() -> None | List[float]
```

Returns most recent finger velocity feedback in degrees per second or None

<a id="hand.Hand.get_fsr"></a>

#### get\_fsr

```python
def get_fsr() -> None | List[float]
```

Returns most recent FSR touch sensor values or None

<a id="hand.Hand.get_tar_position"></a>

#### get\_tar\_position

```python
def get_tar_position() -> None | List[float]
```

Returns hand target positions / last position command or None

<a id="hand.Hand.get_tar_velocity"></a>

#### get\_tar\_velocity

```python
def get_tar_velocity() -> None | List[float]
```

Returns hand target velocities / last velocity command or None

<a id="hand.Hand.get_tar_current"></a>

#### get\_tar\_current

```python
def get_tar_current() -> None | List[float]
```

Returns hand target currents / last current command or None

<a id="hand.Hand.get_tar_duty"></a>

#### get\_tar\_duty

```python
def get_tar_duty() -> None | List[float]
```

Returns hand target duty cycle / last duty command or None

