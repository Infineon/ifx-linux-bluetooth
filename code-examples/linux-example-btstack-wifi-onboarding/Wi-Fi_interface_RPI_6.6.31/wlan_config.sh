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

echo "$config" | sudo tee /etc/wpa_supplicant/wpa_supplicant.conf

# Clean disconnect and kill
sudo ./wpa_cli -i wlan0 disconnect
sudo pkill wpa_supplicant
sleep 1

# Clean leftover control interface if needed
if [ -e /var/run/wpa_supplicant/wlan0 ]; then
  echo "Removing stale control interface..."
  sudo rm /var/run/wpa_supplicant/wlan0
fi
# Bring up the interface
sudo ip link set wlan0 up

# Restart wpa_supplicant
sudo ./wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf

# Request a new IP address
sudo dhclient wlan0


