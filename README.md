squirrel_driver

First you need to install FT17 driver to your machine

How to install the FT17 driver:

Execute:
cd ft17_driver
./install.sh

This script will install the dynamic library libFT17_driver.so inside /usr/lib and all the related headers file in /usr/include

THEN COPY THOSE FILES TO COMMON LIB AND INCLUDE MANUALLY

To run the sensor: 

Prerequisites
- Connect the ethernet cable from the PC to the FT17 sensor.
- Set a manual IP in the 192.168.1.1/24 subnet, e.g. 192.168.1.100 with a subnet mask 255.255.255.0
- Install the FT17_driver as described above

Execute:
rosrun squirrel_sensing_node sensing
Two topics are published:
/wrist
/fingertips

