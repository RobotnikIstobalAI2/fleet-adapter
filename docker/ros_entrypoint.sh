#!/bin/bash
set -e

function load_ros_file() {
  file="${1}"
  echo "loading ${1}"
  if [[ -r "${file}" ]]
  then
    if ! source "${file}"
    then
      echo "could not load ${file}"
      return 1
    fi
  else
    echo "${file} do not exist"
  fi
  return 0
}

function setup_ros() {
  # Load general ros setup
  file="/opt/ros/${ROS_DISTRO}/setup.bash"
  echo "loading ${file}"
  if ! source "${file}"
  then
    echo "could not load ${file}"
    return 1
  fi
  # Load additional ros setups
  OIFS=$IFS
  IFS=':'
  files="${ROS_SETUP_FILES}"
  for file in ${files[@]}
  do
    if ! load_ros_file "${file}"
    then
      IFS=$OIFS
      return 1
    fi
  done
  IFS=$OIFS
  return 0
}

setup_ros
exec "$@"

