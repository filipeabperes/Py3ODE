#!/bin/bash
set -e

# Pass install dir as first argument. Defaults to $HOME/ode
INSTALL_DIR=${1:-${HOME}/ode}

ODE_VERSION=0.16
ODE_DIR=ode-${ODE_VERSION}
ODE_TAR=${ODE_DIR}.tar.gz
BUILD_DIR=ode_build

pip install cython

rm -rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR
wget https://bitbucket.org/odedevs/ode/downloads/${ODE_TAR}
tar -xvf $ODE_TAR
rm $ODE_TAR
cd $ODE_DIR
./configure CFLAGS="$CFLAGS -fPIC" CXXFLAGS="$CXXFLAGS -fPIC" --prefix=${INSTALL_DIR}
make
make install
cd ../..
rm -rf $BUILD_DIR
