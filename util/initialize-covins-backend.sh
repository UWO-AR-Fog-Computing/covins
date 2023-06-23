#!/bin/bash

cd /root/rosworkspace/src/covins/covins_backend/
cd config
if [ ! -f "ORBvoc.txt" ]
then
  unzip ORBvoc.txt.zip
fi

cd /root/rosworkspace/src/covins/covins_backend/
#DBoW2
cd thirdparty/DBoW2
if [ ! -d "build" ]
then
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j$1
fi