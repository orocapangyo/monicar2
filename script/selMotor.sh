#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one of these RPM"
    echo "MOTOR_60RPM, MOTOR_178RPM"
	exit 1
fi

echo "set MOTOR_TYPE to" "$1"
echo 'set TPR in monicar2_localization/param/robot.yaml'
if [ "$1" == "MOTOR_178RPM" ]; then
    sed -i "s/TPR:.*/TPR: 620.0/g" ../monicar2_localization/param/robot.yaml
else
    sed -i "s/TPR:.*/TPR: 1860.0/g" ../monicar2_localization/param/robot.yaml
fi

cd ../arduino/
for d in ./*/; do
	cd "$d";
	for filename in ./*.ino; do
		if [[ $(awk '/MOTOR_TYPE/' "$filename") ]]; then
			echo 'Matched' "$filename"
			sed -i "s/#define MOTOR_TYPE.*/#define MOTOR_TYPE $1/g" "$filename"
		fi
	done
	cd ..
done
