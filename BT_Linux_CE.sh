#!/bin/sh

cwd=$(pwd)
uart_port=/dev/ttyAMA0
app_baudrate=3000000
fw_baudrate=115200
bd_addr=112233445566
is_le_supported_chip=1
fw_file=CYW55560A1_001.002.087.0254.0000_Generic_UART_37_4MHz_fcbga_iPA_dLNA_ANT0.hcd
if [ "$1" = "CYW4373" ]; then
    is_le_supported_chip=0
    fw_file=CYW4373A0_001.001.025.0118.0000_Generic_UART_37_4MHz_wlbga_BU_dLNA.hcd
elif [ "$1" = "CYW43439" ]; then
    is_le_supported_chip=0
    fw_file=CYW4343A2_001.003.016.0063.0000_Generic_UART_26MHz_wlbga_BU_Audio_dl_signed.hcd
elif [ "$1" = "CYW5557X" ]; then
    is_le_supported_chip=1
    fw_file=CYW55560A1_001.002.087.0254.0000_Generic_UART_37_4MHz_fcbga_iPA_dLNA_ANT0.hcd
else
    echo "Unknown Chip ID : $1"
    echo "Valid options:"
    echo "./BT_Linux_CE.sh CYW4373"
    echo "./BT_Linux_CE.sh CYW43439"
    echo "./BT_Linux_CE.sh CYW5557X"
    exit 1
fi
sudo apt-get install git cmake gcc-aarch64-linux-gnu build-essential -y
sleep 1

clear
display_content_full() {
    echo "//=====BT Linux Code Examples=====//"
    echo "       1.  Hello Sensor"
    echo "       2.  SPP"
    echo "       3.  Headset"
    echo "       4.  Find Me Target"
    echo "       5.  Wifi-Onboarding"
    echo "       6.  LE Audio CIS source"
    echo "       7.  LE Audio CIS sink"
    echo "       8.  LE Audio BIS Source"
    echo "       9.  LE Audio BIS Sink"

}
display_content() {
    echo "//=====BT Linux Code Examples=====//"
    echo "       1.  Hello Sensor"
    echo "       2.  SPP"
    echo "       3.  Headset"
    echo "       4.  Find Me Target"
    echo "       5.  Wifi-Onboarding"
}
handle_selection() {
    cd $cwd
    echo $cwd
    is_le_audio=0
    is_src=0
    is_br_edr_audio=0
    is_wifi_app=0
    rm -rf build
    mkdir build
    read -p "Enter your choice: " choice
    case $choice in
        1)
            echo "Hello Sensor"
            example_code=linux-example-btstack-hello-sensor
            ;;
        2)
            echo "linux-example-btstack-spp"
            example_code=linux-example-btstack-spp
            ;;
        3)
            echo "Headset"
            example_code=linux-example-btstack-headset
            is_br_edr_audio=1
            ;;
        4)
            echo "Find Me Target"
            example_code=linux-example-btstack-findme
            ;;
        5)
            echo "Wifi-Onboarding"
            example_code=linux-example-btstack-wifi-onboarding
            is_wifi_app=1
            ;;
        6)
            echo "LE Audio CIS Source"
            example_code=linux-example-btstack-unicast-source
            is_le_audio=1
            is_src=1
           ;;
        7)
            echo "LE Audio CIS Sink"
            example_code=linux-example-btstack-unicast-sink
            is_le_audio=1
            ;;
        8)
            
            echo "LE Audio BIS Source"
            example_code=linux-example-btstack-broadcast-source
            is_le_audio=1
            is_src=1
            ;;
        9)
            echo "LE Audio BIS Sink"
            example_code=linux-example-btstack-broadcast-sink
            is_le_audio=1
            ;;
        *)
            echo "Invalid choice. Please try again."
            handle_selection
            ;;
    esac
    if [ $is_le_audio -eq 1 ]; then
        echo "is_le_supported_chip : $is_le_supported_chip"
        if [ $is_le_supported_chip -eq 0 ]; then
            echo "Invalid choice. Please try again."
            handle_selection
        fi
        if [ -d "liblc3" ]; then
            echo "Skip Cloning Google LC3 as it already exist!"
        else
            echo "Cloning Google LC3"
            git clone https://github.com/google/liblc3 --branch v1.0.3
        fi
        cd code-examples/$example_code/COMPONENT_LC3_CODEC/google_lc3
        sudo chmod 777 ./build_google_lc3.sh
        ./build_google_lc3.sh
        cd  ../../../../
        if [ $is_src -eq 1 ]; then
            cp -r $PWD/code-examples/$example_code/test_audio_files $PWD/build
        fi
        cp -r $PWD/le-audio-profiles-linux/COMPONENT_le_audio_profiles_linux/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/lible_audio_profiles.so $PWD/build
        cp -r $PWD/le-audio-profiles-linux/COMPONENT_gatt_interface_linux/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/libgatt_interface.so $PWD/build
    fi
    if [ $is_br_edr_audio -eq 1 ]; then
        cp -r $PWD/bt-audio-profiles/sbc/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/libsbc.so $PWD/build
    fi
    if [ $is_wifi_app -eq 1 ]; then
        cp -r $PWD/code-examples/$example_code/Wi-Fi_interface_RPI $PWD/build 
    fi
    #Build application
    cd code-examples/$example_code
    rm -rf app_build
    mkdir app_build
    cd app_build
    cmake -DCMAKE_C_COMPILER:PATH=/usr/bin/aarch64-linux-gnu-gcc ../
    make -j8
    chmod +x $example_code
    cd ../../../
    cp -r $PWD/code-examples/$example_code/app_build/$example_code $PWD/build
    rm -rf $PWD/code-examples/$example_code/app_build/
    cp -r $PWD/btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/libbtstack.so $PWD/build
    cp -r $PWD/fw/$fw_file $PWD/build
    cd build
    echo "===Executing "$example_code"==="
    echo "============================"
    echo "============================"
    sudo ./$example_code -c $uart_port -b $app_baudrate -r gpiochip0 3 -f $fw_baudrate -p $fw_file -d $bd_addr
}

# Main script
while true; do
    if [ $is_le_supported_chip -eq 1 ]; then
        display_content_full
    else
        display_content
    fi
    handle_selection
done
