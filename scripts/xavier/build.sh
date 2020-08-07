#!/bin/bash

cmake -DWITH_EXAMPLES=off -DWITH_NETWORK=off -DCMAKE_INSTALL_PREFIX="/opt/aditof" -DCMAKE_PREFIX_PATH="/opt/glog" -DXAVIER=1 ..
make -j$(nproc)
