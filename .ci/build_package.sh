#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: building the package"

ARTIFACTS_FOLDER=$1
BASE_IMAGE=$2

sudo apt-get -y install dpkg-dev

echo "$0: building the package into '$ARTIFACTS_FOLDER'"

mkdir -p $ARTIFACTS_FOLDER

epoch=1
# SHA=$(git rev-parse --short HEAD)
build_flag=$(date +%Y%m%d.%H%M%S)

sed -i "s/(/($epoch:/" ./package/DEBIAN/changelog
sed -i "s/)/.${build_flag})/" ./package/DEBIAN/changelog

dpkg-deb --build --root-owner-group package

dpkg-name package.deb

mv *.deb $ARTIFACTS_FOLDER
