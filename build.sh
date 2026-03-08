#!/bin/bash

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf ../../build/
rm -rf ../../install/
# clear src/CMakeLists.txt if it exists.
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# build
pushd `pwd` > /dev/null
cd ../../
colcon build
popd > /dev/null

popd > /dev/null
