#!/bin/bash

GREEN='\033[0;32m'
PURPULE='\033[0;35m'
NC='\033[0m' # No Color

function CheckIfInstalledAndInstallIt() {
	PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $1 | grep "install ok installed")
	echo "Checking for $1: $PKG_OK"
	if [ "" == "$PKG_OK" ]; then
	  echo $PURPULE"No $1. Setting up $1."$NC
	  sudo apt-get --yes install $1
	fi
}

echo "cloud_stream ros package setup script"

CheckIfInstalledAndInstallIt ros-kinetic-node-manager-fkie
CheckIfInstalledAndInstallIt ros-kinetic-default-cfg-fkie
CheckIfInstalledAndInstallIt ros-kinetic-qt-gui

