#!/bin/bash

# adds rules file for the sensors. See documentation for details

echo -e "SUBSYSTEM==\"video4linux\", ATTR{name}==\"HD Pro Webcam C920\", ATTRS{idProduct}==\"082d\", ATTRS{idVendor}==\"046d\", ATTRS{product}==\"HD Pro Webcam C920\", SYMLINK+=\"camera/logitech\", SUBSYSTEM==\"video4linux\", ATTR{name}==\"USB 2.0 Camera\", ATTRS{idProduct}==\"9230\", ATTRS{idVendor}==\"05a3\", ATTRS{product}==\"USB 2.0 Camera\", SYMLINK+=\"camera/elp\", SUBSYSTEM==\"tty\", ATTRS{idProduct}==\"6001\", ATTRS{idVendor}==\"0403\", ATTRS{product}==\"ARDUINO NANO\", SYMLINK+=\"controller/nano\", SUBSYSTEM==\"tty\", ATTRS{idProduct}==\"01a7\", ATTRS{idVendor}==\"1546\", ATTRS{product}==\"u-blox 7 - GPS/GNSS Receiver\", SYMLINK+=\"gps/g_mouse\", SUBSYSTEM==\"tty\", ATTRS{idProduct}==\"9d0f\", ATTRS{idVendor}==\"1b4f\", ATTRS{product}==\"SFE 9DOF-D21\", SYMLINK+=\"imu/razor_9dof\"" | sudo tee /etc/udev/rules.d/99-device-rules.rules

sudo udevadm control --reload-rules && udevadm trigger




../workspace/src/rplidar_ros/scripts/create_udev_rules.sh

sudo udevadm control --reload-rules && udevadm trigger



