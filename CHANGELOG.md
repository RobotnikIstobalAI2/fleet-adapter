# Changelog

## [Unreleased] - 2023-08-24

### Added

- Added github actions for automatic release and image push
- Added new environment to gain more file grain control of each parameter

### Changed
- Changed to robotnik public `robotnik/ros:humble-{base/builder}-5567def-20230823` ros images as docker image base.
- Now the ros packages are deb generated


## [0.1.1] - 2023-07-18

### Add

- Use a fleet adapter custom package
- Mqtt variables can be configured
- Navigation is considered completed when the distance is less than distance setting in .yaml.
- If the robot position is not receive when the fleet adapter start the system is still running.

### Removed 

- Unused functions

## [0.1.1-rc02] - 2023-06-20

### Add

- Use a fleet adapter custom package
- Mqtt variables can be configured
- Navigation is considered completed when the distance is less than distance setting in .yaml.
- If the robot position is not receive when the fleet adapter start the system is still running.

### Removed 

- Unused functions

## [0.1.0] - 2023-05-04

### Removed

- GPU usage

## [0.1.0-rc01]  -  2023-04-26

### Changed 

-  Change from *galactic* to *humble* version.

### Removed

- Fleet adapter using ROS

### Added 

- Fleet adapter using MQTT
