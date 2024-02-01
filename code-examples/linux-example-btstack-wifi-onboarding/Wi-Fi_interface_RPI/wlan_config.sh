#!/bin/bash
# $ Copyright 2023-YEAR Cypress Semiconductor $

config="# WPA Supplicant with WPA2PSK-AES network security configuration.
ctrl_interface=/var/run/wpa_supplicant
driver_param=use_p2p_group_interface=1p2p_device=1
update_config=1
device_name=RPI-LINUX
config_methods=virtual_push_button physical_display keyboard
interworking=1
sae_pwe=2
sae_groups=19
network={
 ssid=\"$1\"
 key_mgmt=WPA-PSK
 proto=WPA2
 pairwise=CCMP
 psk=\"$2\"
}"

sudo echo "$config" > /etc/wpa_supplicant/wpa_supplicant.conf

sudo wpa_cli -i wlan0 remove_network 0 > _
sudo wpa_cli -i wlan0 reconfigure


