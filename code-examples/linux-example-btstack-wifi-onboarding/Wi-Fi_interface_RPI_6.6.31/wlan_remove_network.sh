#!/bin/bash
# $ Copyright 2023-YEAR Cypress Semiconductor $

sudo ./wpa_cli -i wlan0 disconnect

# Dynamically get network ID
net_id=$(./wpa_cli -i wlan0 list_networks | awk 'NR==2 {print $1}')
sudo ./wpa_cli -i wlan0 remove_network "$net_id"

# Release IP
sudo dhclient -r wlan0
