#!/bin/bash
set -e ; # terminate the script if any command fails

# check if the script is run as root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root. Please use sudo." ;
    exit 1
fi

# load the architecture string ("amd64" or "arm64" is expected)
ARCH=$(dpkg-architecture -qDEB_HOST_ARCH) ;
DEB_HOST_MULTIARCH=$(dpkg-architecture -qDEB_HOST_MULTIARCH) ;

###################################
### CONFIGURATION SECTION START ###
###################################

# selection of sdbus-cpp version
SDBUS_VERSION=2.3.1 ;

# custom package name; it replaces distro package when explicitly installed,
# but it is not the same package so normal system updates do not require it.
PACKAGE_NAME="mrs-libsdbus-c++" ; 

# the final package name
PACKAGE_FILENAME=$PACKAGE_NAME"_"$SDBUS_VERSION"_"$ARCH".deb" ;

# package metadata
PACKAGE_MAINTAINER="Vojtech Vrba <vrba.vojtech@fel.cvut.cz>" ;
PACKAGE_DEPENDS="cmake, dbus-daemon, debhelper-compat, libexpat1-dev, libgmock-dev, libsystemd-dev, pkg-config" ;
PACKAGE_PROVIDES="libsdbus-c++-dev (= $SDBUS_VERSION), libsdbus-c++1 (= $SDBUS_VERSION), libsdbus-c++-bin (= $SDBUS_VERSION), libsdbus-c++-doc (= $SDBUS_VERSION)" ;
PACKAGE_CONFLICTS="libsdbus-c++-dev, libsdbus-c++1, libsdbus-c++-bin, libsdbus-c++-doc" ;
PACKAGE_REPLACES="libsdbus-c++-dev, libsdbus-c++1, libsdbus-c++-bin, libsdbus-c++-doc" ;

###################################
###  CONFIGURATION SECTION END  ###
###################################

# clean before running the script
rm -rf sdbus-cpp* ;

# install pre-requisites
apt-get -y update ;
apt-get -y install git cmake debhelper-compat cmake dbus-daemon libexpat1-dev libgmock-dev libsystemd-dev pkg-config ;

# clone the repository
git clone https://github.com/Kistler-Group/sdbus-cpp.git --branch v$SDBUS_VERSION --depth 1 ;

# enter the sdbus-cpp directory
cd sdbus-cpp ;

# setup build directory
mkdir build ;
cd build ;

# compile everything
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DCMAKE_INSTALL_LIBDIR=lib/$DEB_HOST_MULTIARCH \
    ${OTHER_CONFIG_FLAGS} ;
cmake --build . ;

# stage installation into a package root
PACKAGE_ROOT="$(pwd)/sdbus_package" ;
mkdir -p "$PACKAGE_ROOT/DEBIAN" ;
make DESTDIR="$PACKAGE_ROOT" install ;

# create package control file
cat > "$PACKAGE_ROOT/DEBIAN/control" <<EOT
Package: $PACKAGE_NAME
Version: $SDBUS_VERSION
Section: admin
Priority: optional
Architecture: $ARCH
Maintainer: $PACKAGE_MAINTAINER
Depends: $PACKAGE_DEPENDS
Provides: $PACKAGE_PROVIDES
Conflicts: $PACKAGE_CONFLICTS
Replaces: $PACKAGE_REPLACES
Description: Custom sdbus-cpp build for MRS UAV system
EOT

# expose shared-library dependency metadata for downstream Debian packages
cat > "$PACKAGE_ROOT/DEBIAN/shlibs" <<EOT
libsdbus-c++ 2 $PACKAGE_NAME (>= $SDBUS_VERSION)
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

# create the new deb package
dpkg-deb --build "$PACKAGE_ROOT" "$PACKAGE_FILENAME" ;

# move the deb package to the parent directory
#chmod 644 ./$PACKAGE_FILENAME ;
mv ./$PACKAGE_FILENAME ../../$PACKAGE_FILENAME ;

# create a world-readable copy outside private home directories for apt validation
if [ -d /var/tmp ] ; then
    cp ../../$PACKAGE_FILENAME /var/tmp/$PACKAGE_FILENAME ;
    chmod 644 /var/tmp/$PACKAGE_FILENAME ;
fi

echo "" ;
echo "###### FINISHED PACKAGE INFO START ######" ;
stat ../../$PACKAGE_FILENAME ;
dpkg-deb --info ../../$PACKAGE_FILENAME ;
if [ -f /var/tmp/$PACKAGE_FILENAME ] ; then
    echo "APT validation copy: /var/tmp/$PACKAGE_FILENAME" ;
fi
echo "###### FINISHED PACKAGE INFO END ######" ;
echo "" ;

# terminate successfully
exit 0

