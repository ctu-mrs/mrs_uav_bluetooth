#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

ARTIFACTS_FOLDER=$1
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

mkdir -p "$ARTIFACTS_FOLDER"

echo "$0: building the package bluez with meshing support"
sudo ./.ci/build_bluez_package.sh

echo "$0: building the package sdbus-cpp"
sudo ./.ci/build_sdbus_package.sh

echo "$0: building the package mrs-uav-bluetooth-service"

if [ "$ARCH" = "amd64" ]; then
  dpkg-deb --build --root-owner-group .ci/pkg_service .
else
  echo "$0: skipping mrs-uav-bluetooth-service build on architecture $ARCH"
fi


mv ./*.deb "$ARTIFACTS_FOLDER"

