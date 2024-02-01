#!/bin/bash

rm -rf build
mkdir build
cd build
cmake -DCMAKE_C_COMPILER:PATH=/usr/bin/aarch64-linux-gnu-gcc ../
make

cp ./linux-example-btstack-findme ../artifacts
cd ../
rm -rf build

