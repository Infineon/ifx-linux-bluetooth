#!/bin/sh
make clean -C ../../../../liblc3
make CC=/usr/bin/aarch64-linux-gnu-gcc -C ../../../../liblc3
