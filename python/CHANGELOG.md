# Changelog

All notable changes to this project will be documented in this file.

## [0.2.0] - 2025-06-11
### Hights
- Move simulated response buffer to write thread loop
- Added close method to simulated serial connection

## [0.2.0] - 2025-06-11
### Added
- Moved a bunch of simulated modules into ah_wrapper so people can use the 
simulated hand if installed with pip.
- Mujoco simulator use will still require cloning the repo

## [0.1.7] - 2025-06-11
### Added
- Added simulated position feedback to simulated hand

## [0.1.6] - 2025-06-06
### Added
- Added hot cold status to Observable/Observer class
- Added example of how to use Observable/Observer design pattern

## [0.1.5] - 2025-06-05
### Highlights
- Reworked some of the examples for cleaner shutdowns
- Throw out bad frames introduced by RS485 adapter
- Hand Observer now shares offset FSR readings

## [0.1.4] - 2025-06-04
### Added
- Added Observer / Observable design pattern and made Hand class an Observable.
  see [observer.py](https://github.com/psyonicinc/ability-hand-api/blob/master/python/ah_wrapper/observer.py)
- This will help with ROS2 and other integrations

## [0.1.3] - 2025-06-03
### Highlights
- Proper PyPi release with description and links to website
- Wrapper provides methods for sending Position, Velocity and Torque commands. 
- Wrapper parses incoming touch data (FSR), position, velocity and current
  feedback

## [0.1.0] - 2025-06-03
### Fixed
- PyPi initial release
- Changed ppp_stuffing.PPPUnstuff.unstuff() to be simpler and compatible with
  various RS485 adapters 
