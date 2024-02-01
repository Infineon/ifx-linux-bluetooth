rm -rf build
mkdir build
cd build

export GCC_CROSS_COMPILER=/usr/bin/aarch64-linux-gnu-gcc
cmake -DLC3CODEC=$1 -DCMAKE_C_COMPILER:PATH=$GCC_CROSS_COMPILER ../
make

