#!/bin/bash
set -e ; # terminate the script if any command fails

# check if the script is run as root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root. Please use sudo." ;
    exit 1
fi

# load the architecture string ("amd64" or "arm64" is expected)
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH) ;

###################################
### CONFIGURATION SECTION START ###
###################################

# selection of BlueZ and ELL versions
BLUEZ_VERSION=5.86 ;
ELL_VERSION=0.82 ;

# custom package name; it replaces distro BlueZ when explicitly installed,
# but it is not the same package so normal system updates do not require it.
PACKAGE_NAME="mrs-bluez" ; # better than bluez-mrs

# the final package name
PACKAGE_FILENAME=$PACKAGE_NAME"_"$BLUEZ_VERSION"_"$ARCH".deb" ;

# package metadata
PACKAGE_MAINTAINER="Vojtech Vrba <vrba.vojtech@fel.cvut.cz>" ;
PACKAGE_DEPENDS="libc6, libdbus-1-3, libglib2.0-0, libreadline8, libudev1, kmod, udev, dbus" ;
PACKAGE_PROVIDES="bluez (= $BLUEZ_VERSION), bluez-obexd (= $BLUEZ_VERSION), bluez-hcidump (= $BLUEZ_VERSION), bluez-meshd (= $BLUEZ_VERSION)" ;
PACKAGE_CONFLICTS="bluez, bluez-obexd, bluez-hcidump, bluez-meshd, bluez-test-tools" ;
PACKAGE_REPLACES="bluez, bluez-obexd, bluez-hcidump, bluez-meshd, bluez-test-tools" ;

###################################
###  CONFIGURATION SECTION END  ###
###################################

# clean before running the script
rm -rf bluez* ell* ;

# install pre-requisites (with sources)
# sed -i '/^#\sdeb-src /s/^# *//' "/etc/apt/sources.list" ; # this enables deb-src for apt
apt-get -y update ;
apt-get -y install git libasound2-dev ;
# apt-get -y build-dep bluez ; # this installs packages obtained by: apt-cache showsrc bluez | grep ^Build-Depends
apt-get -y satisfy "debhelper (>= 9), autotools-dev, dh-autoreconf, flex, bison, libdbus-glib-1-dev, libglib2.0-dev (>= 2.28), libcap-ng-dev, udev, libudev-dev, libreadline-dev, libical-dev, check (>= 0.9.8-1.1), systemd, libsystemd-dev, libebook1.2-dev (>= 3.12)" ;

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
    --enable-midi \
    --enable-mesh ;

# compile everything
make ;

# stage installation into a package root
PACKAGE_ROOT="$(pwd)/bluez_package" ;
mkdir -p "$PACKAGE_ROOT/DEBIAN" ;
make DESTDIR="$PACKAGE_ROOT" install ;

# enable experimental mode and disable all plugins in the packaged systemd unit
BLUETOOTH_SERVICE="$PACKAGE_ROOT/usr/lib/systemd/system/bluetooth.service" ;
if [ ! -f "$BLUETOOTH_SERVICE" ] ; then
    BLUETOOTH_SERVICE="$PACKAGE_ROOT/lib/systemd/system/bluetooth.service" ;
fi
if [ -f "$BLUETOOTH_SERVICE" ] ; then
    sed -Ei '/^[[:space:]]*ExecStart=/ {
        /(^|[[:space:]])-E([[:space:]]|$)/! s#$# -E#
        /(^|[[:space:]])-P[[:space:]]+\*([[:space:]]|$)/! s#$# -P *#
    }' "$BLUETOOTH_SERVICE" ;
fi

# create package control file
cat > "$PACKAGE_ROOT/DEBIAN/control" <<EOT
Package: $PACKAGE_NAME
Version: $BLUEZ_VERSION
Section: admin
Priority: optional
Architecture: $ARCH
Maintainer: $PACKAGE_MAINTAINER
Depends: $PACKAGE_DEPENDS
Provides: $PACKAGE_PROVIDES
Conflicts: $PACKAGE_CONFLICTS
Replaces: $PACKAGE_REPLACES
Description: Custom bluez build for MRS UAV system
EOT

# mark files under /etc as configuration so upgrades preserve local edits
if [ -d "$PACKAGE_ROOT/etc" ] ; then
    find "$PACKAGE_ROOT/etc" -type f | sed "s#^$PACKAGE_ROOT##" | sort > "$PACKAGE_ROOT/DEBIAN/conffiles" ;
fi

# generate md5sums for package contents
(
    cd "$PACKAGE_ROOT" ;
    find . -type f ! -path './DEBIAN/*' -print0 | xargs -0 md5sum > ./DEBIAN/md5sums ;
) ;

# create pre-installation package script
cat > "$PACKAGE_ROOT/DEBIAN/preinst" <<'EOT'
#!/bin/sh
set -e

case "$1" in
  upgrade|install)
	install -m 700 -d /var/lib/bluetooth
  ;;
esac

exit 0
EOT
chmod +x "$PACKAGE_ROOT/DEBIAN/preinst" ;

# create post-installation package script
cat > "$PACKAGE_ROOT/DEBIAN/postinst" <<'EOT'
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

        # refresh systemd unit files and start bluetoothd so bluetoothctl works immediately
        if command -v systemctl >/dev/null 2>&1 ; then
            systemctl daemon-reload || true
            systemctl enable bluetooth.service >/dev/null 2>&1 || true
            systemctl restart bluetooth.service || systemctl start bluetooth.service || true
        elif [ -x /etc/init.d/bluetooth ] ; then
            invoke-rc.d bluetooth restart || invoke-rc.d bluetooth start || true
        fi

        ;;
    abort-upgrade|abort-remove|abort-deconfigure|triggered)
    ;;

    *)
        exit 0
    ;;
esac

exit 0
EOT
chmod +x "$PACKAGE_ROOT/DEBIAN/postinst" ;

# create pre-removal package script
cat > "$PACKAGE_ROOT/DEBIAN/prerm" <<'EOT'
#!/bin/sh
set -e

case "$1" in
    remove)
	if command -v systemctl >/dev/null 2>&1 ; then
	    systemctl stop bluetooth.service || true
	elif [ -x /etc/init.d/bluetooth ] ; then
	    invoke-rc.d bluetooth stop || true
	fi

	if [ -d /var/lib/bluetooth ] ; then
	    rm -rf /var/lib/bluetooth
	fi
    ;;
esac

exit 0
EOT
chmod +x "$PACKAGE_ROOT/DEBIAN/prerm" ;

# create the new deb package
dpkg-deb --build "$PACKAGE_ROOT" "$PACKAGE_FILENAME" ;

# move the deb package to the parent directory
#chmod 644 ./$PACKAGE_FILENAME ;
mv ./$PACKAGE_FILENAME ../$PACKAGE_FILENAME ;

# create a world-readable copy outside private home directories for apt validation
if [ -d /var/tmp ] ; then
    cp ../$PACKAGE_FILENAME /var/tmp/$PACKAGE_FILENAME ;
    chmod 644 /var/tmp/$PACKAGE_FILENAME ;
fi

echo "" ;
echo "###### FINISHED PACKAGE INFO START ######" ;
stat ../$PACKAGE_FILENAME ;
dpkg-deb --info ../$PACKAGE_FILENAME ;
if [ -f /var/tmp/$PACKAGE_FILENAME ] ; then
    echo "APT validation copy: /var/tmp/$PACKAGE_FILENAME" ;
fi
echo "###### FINISHED PACKAGE INFO END ######" ;
echo "" ;

# terminate successfully
exit 0

