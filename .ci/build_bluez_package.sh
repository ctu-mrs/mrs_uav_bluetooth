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

# install pre-requisites (with sources)
# sed -i '/^#\sdeb-src /s/^# *//' "/etc/apt/sources.list" ; # this enables deb-src for apt
apt-get -y update ;
apt-get -y install git checkinstall libasound2-dev ;
# apt-get -y build-dep bluez ; # this installs packages obtained by: apt-cache showsrc bluez | grep ^Build-Depends
apt-get -y satisfy "debhelper (>= 9), autotools-dev, dh-autoreconf, flex, bison, libdbus-glib-1-dev, libglib2.0-dev (>= 2.28), libcap-ng-dev, udev, libudev-dev, libreadline-dev, libical-dev, check (>= 0.9.8-1.1), systemd, dh-systemd (>= 1.5), libebook1.2-dev (>= 3.12)" ;


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
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH) ;

checkinstall -D \
    --install=no \
    --fstrans=yes \
    --pkgversion=$BLUEZ_VERSION \
    --pkgname=bluez \
    --arch=$ARCH \
    --requires="libc6, libdbus-1-3, libglib2.0-0, libreadline8, libudev1, kmod, udev, lsb-base, dbus" \
    --provides="bluez \(= $BLUEZ_VERSION\)" \
    --replaces=bluez,bluez-cups,bluez-obexd \
    --maintainer="Vojtech Vrba \<vrba.vojtech\@fel.cvut.cz\>" \
    --nodoc	\
    --default ;

# modify the deb package
PACKAGE_NAME="bluez_"$BLUEZ_VERSION"_"$ARCH".deb" ;

dpkg-deb -x ./bluez_*.deb ./bluez_package ;
dpkg-deb --control ./bluez_*.deb ./bluez_package/DEBIAN ;

sed -i "s/Description:.*/Description: Custom bluez build for MRS UAV system/g" ./bluez_package/DEBIAN/control ;
sed -i "s/Version:.*/Version: $BLUEZ_VERSION/g" ./bluez_package/DEBIAN/control ;

cat > ./bluez_package/DEBIAN/preinst <<EOT
#!/bin/sh
set -e

case "$1" in
  upgrade|install)
	install -m 700 -d /var/lib/bluetooth
  ;;
esac

exit 0
EOT
chmod +x ./bluez_package/DEBIAN/preinst ;

cat > ./bluez_package/DEBIAN/postinst <<EOT
#!/bin/sh
set -e

case "$1" in
    configure)
        # create bluetooth group if not already present
        if ! getent group bluetooth > /dev/null; then
            addgroup --quiet --system bluetooth
        fi

        # reload dbus config file
        if [ -x /etc/init.d/dbus ]; then
            invoke-rc.d dbus force-reload || true
        fi

        ;;
    abort-upgrade|abort-remove|abort-deconfigure)
    ;;

    *)
        echo "postinst called with unknown argument: '$1'" >&2
        exit 0
    ;;
esac

exit 0
EOT
chmod +x ./bluez_package/DEBIAN/postinst ;

cat > ./bluez_package/DEBIAN/prerm <<EOT
#!/bin/sh
set -e

case "$1" in
    remove)
	if [ -d /var/lib/bluetooth ] ; then
	    rm -rf /var/lib/bluetooth
	fi
    ;;
esac

exit 0
EOT
chmod +x ./bluez_package/DEBIAN/prerm ;

dpkg -b ./bluez_package $PACKAGE_NAME ;

# move the deb package to the parent directory
chmod 777 ./$PACKAGE_NAME ;
cp ./$PACKAGE_NAME .. ;

echo "" ;
echo "###### FINISHED PACKAGE INFO START ######" ;
stat ./$PACKAGE_NAME ;
dpkg-deb --info ./$PACKAGE_NAME ;
echo "###### FINISHED PACKAGE INFO END ######" ;
echo "" ;

# terminate successfully
exit 0

