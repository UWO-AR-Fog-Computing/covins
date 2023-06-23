#!/bin/bash

cd /root/rosworkspace/src/covins/orb_slam3/Thirdparty/DBoW2
if [ ! -d "build" ]
then
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j$1
fi

cd /root/rosworkspace/src/covins/orb_slam3/Thirdparty/g2o
if [ ! -d "build" ]
then
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j$1
fi

cd /root/rosworkspace/src/covins/orb_slam3/Vocabulary
if [ ! -f "ORBvoc.txt" ]
then
  tar -xf ORBvoc.txt.tar.gz
fi

cd /root/rosworkspace/src/covins/orb_slam3
if [ ! -d "build" ]
then
  mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$1