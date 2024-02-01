#!/bin/bash
# $ Copyright 2023-YEAR Cypress Semiconductor $

wpa_cli -i wlan0 scan_results|awk -F '\t' '{print $4, $5}'
