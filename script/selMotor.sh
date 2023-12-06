#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one of these RPM"
    echo "60, 178"
	exit 1
fi

if [ "$1" == "178" ]; then
    echo "178RPM motor set"
    sed -i 's/MOTOR_TYPE MOTOR_60RPM/MOTOR_TYPE MOTOR_178RPM/g' ../arduino/motorController32Test.ino
    sed -i 's/MOTOR_TYPE MOTOR_60RPM/MOTOR_TYPE MOTOR_178RPM/g' ../arduino/motorEncLedMpuRos32.ino
    sed -i 's/1860.0/620.0/g' ../monicar2_localization/param/robot.yaml
else
    echo "60RPM motor set"
    sed -i 's/MOTOR_TYPE MOTOR_178RPM/MOTOR_TYPE MOTOR_60RPM/g' ../arduino/motorController32Test.ino
    sed -i 's/MOTOR_TYPE MOTOR_178RPM/MOTOR_TYPE MOTOR_60RPM/g' ../arduino/motorEncLedMpuRos32.ino
    sed -i 's/620.0/1860.0/g' ../monicar2_localization/param/robot.yaml
fi 