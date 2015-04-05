#!/bin/sh

TEST_IP="8.8.8.8"

# Test the network connection.
ping -c1 -w3 ${TEST_IP} > /dev/null 2>&1
if [ $? -ne 0 ]; then
#	NOW="$(date)"
#	echo "${NOW} Ping timeout, restarting connman" >> /home/root/connman-watchdog.log
	systemctl restart connman
fi

exit 0

