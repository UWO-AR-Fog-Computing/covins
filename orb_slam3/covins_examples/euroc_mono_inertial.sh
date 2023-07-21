#!/bin/bash
pathDatasets='/root/rosworkspace/data/datasets'
#------------------------------------
# Monocular-Inertial Examples
source /root/rosworkspace/devel/setup.bash
echo "Launching $1 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc $pathDatasets/ORBvoc.txt $pathDatasets/EuRoC/EuRoC-Monocular-Inertial.yaml $pathDatasets/EuRoC/$1 $pathDatasets/EuRoC/$1/$1.txt dataset-$1-mono-inertial

#gdb --args ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi
