#!/bin/bash

DEVICE_PATH="/dev/ttyUSB0"

if [ -e "$DEVICE_PATH" ]; then
    CURRENT_PERMISSIONS=$(stat -c %a "$DEVICE_PATH")
    if [ "$CURRENT_PERMISSIONS" != "666" ]; then
        chmod 0666 "$DEVICE_PATH"
        echo "Permissions for $DEVICE_PATH set to 0666"
    else
        echo "Permissions for $DEVICE_PATH are already 0666"
    fi
fi