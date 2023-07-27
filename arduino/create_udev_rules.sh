#!/bin/bash

echo "remap the device serial port(ttyUSBX) to arduinoNano"
sudo cp arduinoNano.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish"