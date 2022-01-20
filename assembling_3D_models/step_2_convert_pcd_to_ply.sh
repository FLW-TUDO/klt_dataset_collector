#!/bin/bash

cd $1
for file in *; do pcl_pcd2ply -format 0 "$file" ${file%????}.ply ; done
sed -i '11,33d' *.ply
