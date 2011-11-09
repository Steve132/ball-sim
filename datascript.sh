#!/bin/bash

scale_max=128
numspheres_max=2048
threads_max=8
x=5
y=3
z=5

for (( i=1; i <= $threads_max; i*=2))
do
    for (( j=1; j <= $scale_max+1; j*=2))
    do
	cur_x=$(($x*$j))
	cur_y=$(($y*$j))
	cur_z=$(($z*$j))
	for (( k=1; k <= $numspheres_max+1; k*=2))
	do
	    echo "Threads = $i"
	    echo "Scale = $j"
	    echo "NumSpheres = $k"
	    echo ""
            /home/narnold/Classes/COP6616/Project/bin/headless_sim -x $cur_x -y $cur_y -z $cur_z -n $k -t $i
            echo ""
	done
    done
done
