#!/bin/bash
echo ""
echo " Copying udev rules..."
echo ""
sudo cp `rospack find kclhand_control`/udev/metahand_usb.rules /etc/udev/rules.d
echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart