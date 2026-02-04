# AIROC™ Bluetooth: Linux Bluetooth Wifi shared sdio driver

This source code is the linux driver for Linux Host application communicate to Bluetooth chip from SDIO interface through Wifi driver using and only use at AIROC™ Wi-Fi & Bluetooth® combo chip.

## Requirements

- Linux kernel header
- Programming language: C
- AIROC™ Wi-Fi & Bluetooth® combo chip
- AIROC™ Wi-Fi & Bluetooth® combo chip Bluetooth® Firmware file (*.hex*).
- IFX-linux-wireless Firmware
- IFX-linux-wireless WIFI driver
    ```
    https://github.com/Infineon/ifx-linux-wireless
    ```
    Find the same BTS_VERSION in wifi driver README

## Support Chips

- CYW55513
- CYW43022

## What is in this release

- linux bt-shared-sdio driver source code
- BT combo chip Firmware

## BTS Version
- BTS_VERSION: 1.1.1

## Description

This bt-shared-sdio driver must work with specific wifi driver and insert driver after wifi driver.
BTS_VERSION: this version need same as Wifi driver use.

## Instructions

- Build driver

    In target linux host, install build-essential and linux-header-(version)

    ```
    cd linux-bt-shared-sdio
    make TARGET_ARCH=x86_64 FMAC_BUILD=TRUE DEBUG_PORT=TRUE
    ```
    after build success will generate btttysdio.ko

- insert driver

    Change to root
    ```bash
    su
    ```

    After insert wifi driver success.
    ```bash
    cd linux-bt-shared-sdio
    insmod $PWD/btttysdio.ko btfw=$PWD/BT_FW/CYW43012C1_003.002.024.0036.0000_Generic_SDIO_37_4MHz_wlbga_ref3_dLNA_dl_signed.hex log_level=0xff

    chmod 777 /dev/ttySDIO
    ```

    check driver insert success and sdio device node create
    ```bash
    ls -l /dev/ttySDIO
    ```
    crwxrwxrwx. 1 root root 10, 60  24 13:29 /dev/ttySDIO

- Use
    BT firmware will start download when first time open /dev/ttySDIO port
    use the MBT tool in https://github.com/Infineon/mbt clone it and follow the readme to build it.

    setup the MBT port env variable
    ```bash
    export MBT_TRANSPORT=/dev/ttySDIO
    ```
    use mbt to open the port and do reset
    ```bash
    ./mbt reset
    [MBT_TRANSPORT: /dev/ttySDIO]
    Wait Controller Detect CTS low
    Init UART ..........
    tx: (4 bytes)
    01 03 0c 00
    rx: (7 bytes)
    04 0e 04 01 03 0c 00
    ```

- Debug
    use dmesg -w to check the message from bt-shared-sdio driver

## Test Environment
    - kernel-release 5.4.21-100.fc30.x86_64
