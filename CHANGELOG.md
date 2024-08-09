# Changelog

## Unreleased

### Added
- Added dispatch action package
- Added door adapter package
- Added more retrys to the connection in `robotnik_rmf_fleet_adapter_mqtt/RobotClientAPI.py`.
- Added use of ros mirror if it's reachable
- Added `container/compose` folder with compose files.
- Added `container/environment` folder with the environment variables.
- Added the following environment variables in `container/environment/compose`:
    - `IMAGE_BASE_VERSION` to modify the default image base version for building the container.
    - `IMAGE_NAME` to change default value of image name for building the container.
    - `REGISTRY_PROJECT` to change default value of registry project for building the container (using a private `project-robotnik` or public `project-docker-hub` registry project).
    - `BUILDER_TYPE` to override or not git code with local changes.
    - `ROS_MIRROR` is a ros apt mirror to increase the download speed.
    - `REGISTRY_BASE` to change default value of registry base name for downloading images without local building.
    - `DOCKER_ROS_DISTRO` to specify the ROS distribution of the container.
    - `VERSION` to indicate the repository version (branch or tag).

### Changed
- Updated `README.md`, `CustomMission.md` and `CHANGELOG.md` files.
- Updated the version of the `package.xml` and `setup.py` files to `0.2.2`.
- Changed to robotnik public `robotnik/ros:humble-base-0.5.0` ros images as docker image base.
- Changed to robotnik public `robotnik/ros:humble-builder-0.5.0` ros images as package builder intermediate image.
- Use of `apt-fast` for parallel downloading for apt in the `Dockerfile`.
- Use of docker compose environment files for unified variables.
- New docker compose structure with, include, override and merge features (consult docker compose documention for futher information).
    - Compose files with content are located in `container/compose`, environment variables for the docker compose splitted and assigned to included.
    - Use yaml of anchors and links to avoid redefinition in the same file.
    - Refactor from `docker-compose.yaml` to `compose.yaml` to future compatibility with podman.
    - Environment variables are located in `container/environment`.

### Fixed
- Refactor from ci.yaml (continuous integration) to cd.yaml (continuous delivery).
- Refractor from `fleet-adapter` to `robotnik_rmf_fleet_adapter` repository name.
- Refractor from `fleet_adapter_mqtt` to `robotnik_rmf_fleet_adapter_mqtt` package.
- Fixed flux and minor format corrections in `robotnik_rmf_fleet_adapter_mqtt/RobotClientAPI.py`.
- Fixed `plan_id` and minor format corrections in `robotnik_rmf_fleet_adapter_mqtt/RobotCommandHandle.py`.
- Fixed `plan_id` and minor format corrections in `robotnik_rmf_fleet_adapter_mqtt/RobotDelivery.py`.

### Removed
- Unused unused modules.

## [0.2.0] - 2023-12-04

### Added
- Added delivery mission
- Added ingestor
- Added filter waypoints

### Changed
- Modified navigation completed
- Changed robot speed in config file
- Improved github actions workflows
- Changed structure to be more clear 


## [0.1.2] - 2023-08-24

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
