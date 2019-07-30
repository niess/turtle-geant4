#!/bin/bash -x

mkdir -p build
cd build
cmake ..
make install
make run-g4turtle
