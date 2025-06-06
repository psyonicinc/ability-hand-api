# Changelog

All notable changes to this project will be documented in this file.

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
