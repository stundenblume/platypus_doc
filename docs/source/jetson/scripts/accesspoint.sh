#!/bin/bash

pid=$(pgrep -x wpa_supplicant)	#kill wpa_supplicant if it is already running

if [ ! -z "$pid" ]; then
  kill -sigterm $pid
fi

pid=$(pgrep -x dhclient)	#kill dhclient if it is already running

if [ ! -z "$pid" ]; then
  kill -sigterm $pid
fi

pid=$(pgrep -x hostapd)	#kill Hostapd

if [ ! -z "$pid" ]; then
  kill -sigterm $pid
fi

sleep 1
hostapd -B /etc/hostapd/hostapd.conf

rv=$(echo $?)	# checking return status hostapd

if [ $rv -ne 0 ]; then
  echo "could not start hostapd"	
  exit 1
else
  echo "Access Point mode started"
fi

exit 0
