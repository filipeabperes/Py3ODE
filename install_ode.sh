#!/bin/bash
set -e

ODE_VERSION=0.13
ODE_DIR=ode-${ODE_VERSION}
ODE_TAR=${ODE_DIR}.tar.bz2
BUILD_DIR=ode_build

rm -rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR
wget https://downloads.sourceforge.net/project/opende/ODE/${ODE_VERSION}/${ODE_TAR}
tar -xvf $ODE_TAR
rm $ODE_TAR
cd $ODE_DIR
./configure CFLAGS="$CFLAGS -fPIC" CXXFLAGS="$CXXFLAGS -fPIC"
make
make install
cd ../..
rm -rf $BUILD_DIR
