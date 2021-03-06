# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [3.0.0] - 2021-08-02
### Added
- Added support for the Parakeet ProE sensor
- Added support for a resample filter as part of the Parakeet ProE sensor

### Modified
- Bump to using parakeet-sdk v3.0.0
- [BREAKING] Added required runtime parameter 'sensorType'
- Added optional runtime parameter 'extraLatencyDelay_ns'

## [2.0.0] - 2021-07-12
### Added
- Allow modification of laser scan frame id
- Now installable via catkin_make install
### Modified
- Changed the ROS LaserScan message fields to contain the proper information
- [BREAKING] Changed how the Parakeet-SDK project should be installed for the ROS node to be aware of it
- Moved published topics and parameters to under the global ROS namespace
- Modified CMake, when using find_package, to find the latest version of parakeet-sdk which can be found
- Updated copyright action to call the action from inside the parakeet-devtools repo
- Bumped to parakeet-sdk v2.0.0

## [1.0.1] - 2021-06-21
### Changed
- Support Parakeet SDK versions beyond 1.0.0
- Properly label units of measure on variables

## [1.0.0] - 2021-06-07
### Added
- Publication of LaserScan data to a ROS topic from a Parakeet Pro, using the parakeet-sdk 
