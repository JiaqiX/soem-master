#!/bin/bash
###
 # @Descripttion: 
 # @Author: editorxu
 # @version: 
 # @Date: 2024-02-29 14:17:00
 # @LastEditors: editorxu
 # @LastEditTime: 2024-03-04 18:57:12
### 

cmake -B build -DCMAKE_BUILD_TYPE=Debug

if [ $? -ne 0 ]; then
    echo "cmake failed"
    exit 1
fi

# make
cd build
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "make failed"
    exit 1
fi

# install
if [ -d install ]; then
    rm -rf install
fi
