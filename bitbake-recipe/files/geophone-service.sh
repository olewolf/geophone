#!/bin/sh

DEVICE=/dev/ttyATM
LOG_FILE=/var/lib/geophone/geophone.log

. /etc/default/geophone

# If only the device class is specified, that is, with no specific device
# number, then use the first device number found.
DEVICE_NUMBER="$( echo $PORT | sed -n -e "s|/dev/[a-zA-Z]\+\([0-9]\+\)|\1|p" )"
if [[ "z${DEVICE_NUMBER}" = "z" ]]; then
	DEVICE_PATH="$( ls -1 ${DEVICE}* | sort | head -1 )" >/dev/null 2>/dev/null
else
	DEVICE_PATH=${DEVICE}
fi

if [[ "z${DEVICE_PATH}" != "z" ]]; then
	/usr/bin/geophone-log-serial ${DEVICE_PATH} >> ${LOG_FILE}
else
	sleep 1
fi
