#!/bin/sh
path_to_catkin_devel="`echo $CMAKE_PREFIX_PATH | cut -d':' -f1`/lib"
path_to_local_steamvr_shared_libraries="$HOME/.local/share/Steam/ubuntu12_32/steam-runtime/amd64/usr/lib/x86_64-linux-gnu/libvulkan*"
echo "\n\nCopy steamvr libvulkan* files." 
echo "\nORIGIN: $path_to_local_steamvr_shared_libraries"
echo "GOAL: $path_to_catkin_devel"
echo "\n"
cp $path_to_local_steamvr_shared_libraries $path_to_catkin_devel
