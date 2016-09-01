#!/bin/bash

INSTALLED_DIR=$(rospack find robotino_api2)/INSTALLED

echo "Script for installing robotino-api2"

if [ -e $INSTALLED_DIR ]; then
	echo  "robotino-api2 is already installed."
else
	wget -qO - http://packages.openrobotino.org/keyFile | sudo apt-key add -
	sudo su
	echo "deb http://packages.openrobotino.org/trusty trusty main" > /etc/apt/sources.list.d/openrobotino.list
	sudo apt-get update
	sudo apt-get install rec-rpc robotino-common robotino_daemons robotino-api2 robotino-examples robotino3-firmware
	touch $INSTALLED_DIR
	echo "Installed robotino-api2."
fi