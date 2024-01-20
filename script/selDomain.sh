#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: your preferred DOMAINID"
	exit 1
fi

echo "set DOMAINID to" "$1"

cd ../arduino/
for d in ./*/; do
	cd "$d";
	for filename in ./*.ino; do
		if [[ $(awk '/DOMAINID/' "$filename") ]]; then
			echo 'Matched' "$filename"
			sed -i "s/#define DOMAINID.*/#define DOMAINID $1/g" "$filename"
		fi
	done
	cd ..
done
