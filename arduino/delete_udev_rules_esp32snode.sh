#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to arduinoNano"
echo "sudo rm   /etc/udev/rules.d/esp32sNodemcu.rules"
sudo rm   /etc/udev/rules.d/esp32sNodemcu.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"