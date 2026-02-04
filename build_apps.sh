#!/bin/bash
base_dir=`git rev-parse --show-toplevel`
app_list=(
    linux-example-btstack-headset \
    linux-example-btstack-hello-sensor \
    linux-example-btstack-wifi-onboarding \
    linux-example-btstack-wakeonle \
    linux-example-btstack-findme \
    linux-example-btstack-spp \
    linux-example-btstack-le-audio-headset \
    linux-example-btstack-le-audio-player
)

if [[ -n "$1" ]]; then
    CMAKE_C_COMPILER="$base_dir/ARM_GNU-A/Linux64/bin/aarch64-none-linux-gnu-gcc"
    CMAKE_PATH="$base_dir/cmake/Linux64/bin/cmake"
else
    CMAKE_C_COMPILER="/usr/bin/aarch64-linux-gnu-gcc"
    CMAKE_PATH="cmake"
fi

for app in "${app_list[@]}"; do
    printf "\n\n\t\t ----- building $app ----- \n\n"
    pushd code-examples/$app > /dev/null
    mkdir temp
    pushd temp > /dev/null
    $CMAKE_PATH -DCMAKE_C_COMPILER:PATH=$CMAKE_C_COMPILER ../
    make -j
    if [ $? == 0 ]
    then
	    printf "\t\t --- building $app SUCCESS ---- \n\n"
    else
        printf "\t\t --- building $app FAILED ----- \n\n"
	    exit 1
    fi
    popd > /dev/null
    popd > /dev/null
	rm -rf ./code-examples/$app/temp/
done
rm -rf ./liblc3/build ./liblc3/bin

exit 0
