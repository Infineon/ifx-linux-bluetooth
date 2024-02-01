app_list=(linux-example-btstack-broadcast-sink linux-example-btstack-broadcast-source linux-example-btstack-findme linux-example-btstack-headset linux-example-btstack-hello-sensor linux-example-btstack-spp linux-example-btstack-unicast-sink linux-example-btstack-unicast-source linux-example-btstack-wifi-onboarding)
rm -rf deploy
mkdir deploy

if [[ -n "$1" ]]; then
    CMAKE_C_COMPILER="./ARM_GNU-A/Linux64/bin/aarch64-none-linux-gnu-gcc"
    CMAKE_PATH="./cmake/Linux64/bin/cmake"
else
    CMAKE_C_COMPILER="/usr/bin/aarch64-linux-gnu-gcc"
    CMAKE_PATH="cmake"
fi

for app in "${app_list[@]}"; do
    printf "\n\n\t\t ----- building $app ----- \n\n"
    cd code-examples/$app
    mkdir temp
    cd temp
    $CMAKE_PATH -DCMAKE_C_COMPILER:PATH=$CMAKE_C_COMPILER ../
    make -j
    if [ $? == 0 ]
    then
	    printf "\t\t --- building $app SUCCESS ---- \n\n"
    else
        printf "\t\t --- building $app FAILED ----- \n\n"
	    exit 1
    fi
    cd ../../../
	cp -r ./code-examples/$app/temp/$app ./deploy
	rm -rf ./code-examples/$app/temp/
done
cp -r ./btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/libbtstack.so ./deploy
cp -r ./code-examples/linux-example-btstack-unicast-source/test_audio_files ./deploy
cp -r ./le-audio-profiles-linux/COMPONENT_le_audio_profiles_linux/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/lible_audio_profiles.so ./deploy
cp -r ./le-audio-profiles-linux/COMPONENT_gatt_interface_linux/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/libgatt_interface.so ./deploy
cp -r ./bt-audio-profiles/sbc/COMPONENT_ARMv8_LINUX/COMPONENT_GCC/libsbc.so ./deploy
cp -r ./code-examples/linux-example-btstack-wifi-onboarding/Wi-Fi_interface_RPI ./deploy 
cp -r ./fw/* ./deploy/
zip -rmT linux_apps.zip deploy
exit 0
