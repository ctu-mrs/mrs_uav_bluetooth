#!/bin/bash

# selection of BlueZ and ELL versions
export BLUEZ_VERSION=5.82 ;
export ELL_VERSION=0.76 ;

# terminate the script if any command fails
set -e ;

# check if the script is run as root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root. Please use sudo." ;
    exit 1
fi

# clean before running the script
rm -rf bluez* ell* ;

# install pre-requisites
apt-get -y install git checkinstall libcups2-dev libasound2-dev ;
apt-get -y build-dep bluez bluez-cups ;

# clone the bluez and ell repositories
git clone https://github.com/bluez/bluez.git --branch $BLUEZ_VERSION --depth 1 ;
git clone https://git.kernel.org/pub/scm/libs/ell/ell.git --branch $ELL_VERSION --depth 1 ;

# enter the bluez directory
cd bluez ;

# recover files (configure.ac etc.) 
./bootstrap ;

# configure the bluez installation
./configure \
    --prefix=/usr \
    --mandir=/usr/share/man \
    --sysconfdir=/etc \
    --localstatedir=/var \
    --disable-cups \
    --enable-library \
    --enable-testing \
    --enable-experimental \
    --enable-deprecated \
    --enable-external-plugins \
    --enable-nfc \
    --enable-sap \
    --enable-health \
    --enable-midi \
    --enable-mesh ;

# compile everything
make ;

# create the deb package by capturing "make install" effects
checkinstall -D \
    --install=no \
    --fstrans=yes \
    --pkgversion=$BLUEZ_VERSION \
    --pkgname=bluez \
    --arch=all \
    --requires=kmod,udev,dbus-system-bus \
    --replaces=bluez,bluez-cups,bluez-obexd \
    --maintainer="Vojtech Vrba \<vrba.vojtech\@fel.cvut.cz\>" \
    --nodoc	\
    --default ;

# move the deb package to the parent directory
chmod 777 bluez_*.deb ;
cp bluez_*.deb .. ;

# here you can check the package with: dpkg-deb --info ./bluez_*.deb

# terminate successfully
exit 0

