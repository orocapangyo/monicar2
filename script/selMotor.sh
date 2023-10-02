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
    sed -i 's/1860.0/620.0/g' ../monicar2_localization/param/initPose0.yaml
    sed -i 's/1860.0/620.0/g' ../monicar2_localization/param/initPose1.yaml
else
    echo "60RPM motor set"
    sed -i 's/620.0/1860.0/g' ../monicar2_localization/param/initPose0.yaml
    sed -i 's/620.0/1860.0/g' ../monicar2_localization/param/initPose1.yaml
fi 