#!/bin/bash
set -e ; # terminate the script if any command fails

# Dependency installation needs root. Builders with the prerequisites already
# installed may set MRS_BLUEZ_SKIP_DEPENDENCIES=1 and package as an ordinary user.
if [ "${MRS_BLUEZ_SKIP_DEPENDENCIES:-0}" != "1" ] && [ "$(id -u)" -ne 0 ]; then
    echo "Run as root, or set MRS_BLUEZ_SKIP_DEPENDENCIES=1 when dependencies are installed." ;
    exit 1
fi

# load the architecture string ("amd64" or "arm64" is expected)
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH) ;

###################################
### CONFIGURATION SECTION START ###
###################################

# selection of BlueZ and ELL versions
BLUEZ_VERSION=5.87 ;
ELL_VERSION=0.83 ;
# Keep the BlueZ package version at 5.87 and include the audited Mesh changes
# in that package selection.
PACKAGE_VERSION="${BLUEZ_VERSION}" ;
PATCH_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/patches" ;
OUTPUT_DIR="$(pwd)" ;

# This custom package explicitly replaces distribution BlueZ while retaining an
# independent package name for ordinary distribution update resolution.
PACKAGE_NAME="mrs-bluez" ; # better than bluez-mrs

# the final package name
PACKAGE_FILENAME=$PACKAGE_NAME"_"$PACKAGE_VERSION"_"$ARCH".deb" ;

# package metadata
PACKAGE_MAINTAINER="Vojtech Vrba <vrba.vojtech@fel.cvut.cz>" ;
PACKAGE_DEPENDS="libc6, libdbus-1-3, libglib2.0-0, libjson-c5, libreadline8, libssl3t64, libudev1, kmod, udev, dbus" ;
PACKAGE_PROVIDES="bluez (= $BLUEZ_VERSION), bluez-obexd (= $BLUEZ_VERSION), bluez-hcidump (= $BLUEZ_VERSION), bluez-meshd (= $BLUEZ_VERSION)" ;
PACKAGE_CONFLICTS="bluez, bluez-obexd, bluez-hcidump, bluez-meshd, bluez-test-tools" ;
PACKAGE_REPLACES="bluez, bluez-obexd, bluez-hcidump, bluez-meshd, bluez-test-tools" ;

###################################
###  CONFIGURATION SECTION END  ###
###################################

# Isolate each build in its own disposable directory and preserve the caller tree.
BUILD_DIR="$(mktemp -d /tmp/mrs-bluez-package.XXXXXX)" ;
cd "$BUILD_DIR" ;

# Install prerequisites unless the caller has already provisioned the builder.
if [ "${MRS_BLUEZ_SKIP_DEPENDENCIES:-0}" != "1" ] ; then
    # sed -i '/^#\sdeb-src /s/^# *//' "/etc/apt/sources.list" ; # enables deb-src for apt
    apt-get -y update ;
    apt-get -y install git libasound2-dev libjson-c-dev libssl-dev python3-docutils ;
    # apt-get -y build-dep bluez ; # packages from: apt-cache showsrc bluez | grep ^Build-Depends
    apt-get -y satisfy "debhelper (>= 9), autotools-dev, dh-autoreconf, flex, bison, libdbus-glib-1-dev, libglib2.0-dev (>= 2.28), libcap-ng-dev, udev, libudev-dev, libreadline-dev, libical-dev, check (>= 0.9.8-1.1), systemd, libsystemd-dev, libebook1.2-dev (>= 3.12)" ;
fi

# clone the bluez and ell repositories
git clone https://github.com/bluez/bluez.git --branch $BLUEZ_VERSION --depth 1 ;
git clone https://git.kernel.org/pub/scm/libs/ell/ell.git --branch $ELL_VERSION --depth 1 ;

# Enter the BlueZ directory and apply the audited daemon fixes in order.
# A changed upstream context fails the build and preserves the complete audited set.
cd bluez ;
git apply --check "$PATCH_ROOT/bluez-adapter-mode-completion.patch" ;
git apply "$PATCH_ROOT/bluez-adapter-mode-completion.patch" ;
git apply --check "$PATCH_ROOT/bluez-mesh-radio-scheduler.patch" ;
git apply "$PATCH_ROOT/bluez-mesh-radio-scheduler.patch" ;
git apply --check "$PATCH_ROOT/bluez-mesh-sar-queue.patch" ;
git apply "$PATCH_ROOT/bluez-mesh-sar-queue.patch" ;
git apply --check "$PATCH_ROOT/bluez-mesh-local-pb-adv.patch" ;
git apply "$PATCH_ROOT/bluez-mesh-local-pb-adv.patch" ;
git apply --check "$PATCH_ROOT/bluez-mesh-joined-provisioner-keyring.patch" ;
git apply "$PATCH_ROOT/bluez-mesh-joined-provisioner-keyring.patch" ;
git apply --check "$PATCH_ROOT/bluez-mesh-reconcile-local-devkey.patch" ;
git apply "$PATCH_ROOT/bluez-mesh-reconcile-local-devkey.patch" ;
git apply --check "$PATCH_ROOT/bluez-mesh-userspace-aes-ccm.patch" ;
git apply "$PATCH_ROOT/bluez-mesh-userspace-aes-ccm.patch" ;
git apply --check "$PATCH_ROOT/bluez-mesh-sequence-reservation.patch" ;
git apply "$PATCH_ROOT/bluez-mesh-sequence-reservation.patch" ;

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
Version: $PACKAGE_VERSION
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

        # Preserve bonds, Mesh keys, and replay state across package removal.
        # Administrators manage individual stored identities explicitly.
    ;;
esac

exit 0
EOT
chmod +x "$PACKAGE_ROOT/DEBIAN/prerm" ;

# create the new deb package
# Unprivileged audit builds must still install system files as root-owned.
dpkg-deb --root-owner-group --build "$PACKAGE_ROOT" "$PACKAGE_FILENAME" ;

# move the deb package to the parent directory
#chmod 644 ./$PACKAGE_FILENAME ;
mv "./$PACKAGE_FILENAME" "$OUTPUT_DIR/$PACKAGE_FILENAME" ;

# Create a convenient apt-validation copy when the destination is writable.
# Treat an existing root-owned /var/tmp copy as an optional validation artifact
# after the unprivileged package build has completed successfully.
APT_VALIDATION_COPY="" ;
if [ -d /var/tmp ] && cp "$OUTPUT_DIR/$PACKAGE_FILENAME" "/var/tmp/$PACKAGE_FILENAME" 2>/dev/null ; then
    chmod 644 "/var/tmp/$PACKAGE_FILENAME" || true ;
    APT_VALIDATION_COPY="/var/tmp/$PACKAGE_FILENAME" ;
else
    echo "Skipping optional /var/tmp copy; package remains in $OUTPUT_DIR." ;
fi

echo "" ;
echo "###### FINISHED PACKAGE INFO START ######" ;
stat "$OUTPUT_DIR/$PACKAGE_FILENAME" ;
dpkg-deb --info "$OUTPUT_DIR/$PACKAGE_FILENAME" ;
echo "Build sources retained for inspection: $BUILD_DIR" ;
if [ -n "$APT_VALIDATION_COPY" ] ; then
    echo "APT validation copy: $APT_VALIDATION_COPY" ;
fi
echo "###### FINISHED PACKAGE INFO END ######" ;
echo "" ;

# terminate successfully
exit 0
