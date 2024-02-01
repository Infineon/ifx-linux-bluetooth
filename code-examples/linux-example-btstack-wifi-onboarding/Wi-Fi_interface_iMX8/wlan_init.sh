#!/bin/bash
# $ Copyright 2023-YEAR Cypress Semiconductor $

config="# WPA Supplicant with WPA2PSK-AES network security configuration.
ctrl_interface=/var/run/wpa_supplicant
driver_param=use_p2p_group_interface=1p2p_device=1
update_config=1
device_name=iMX-LINUX
config_methods=virtual_push_button physical_display keyboard
interworking=1
sae_pwe=2
sae_groups=19
network={
 ssid=\"userSSID\"
 key_mgmt=WPA-PSK
 proto=WPA2
 pairwise=CCMP
 psk=\"userPASSWORD\"
}"

echo "$config" > /etc/wpa_supplicant.conf

wpa_supplicant -B -c /etc/wpa_supplicant.conf -i wlan0 > _
sleep 1
wpa_cli -i wlan0 remove_network 0 > _
