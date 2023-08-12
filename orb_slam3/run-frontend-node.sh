#!/bin/bash
pathDatasets='/root/rosworkspace/data/datasets'
source /root/rosworkspace/devel/setup.bash
rosrun orb_slam3 orbslam3_frontend_node $pathDatasets/ORBvoc.txt $pathDatasets/EuRoC/EuRoC-Monocular.yaml