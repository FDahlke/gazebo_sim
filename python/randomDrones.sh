#!/bin/bash

for (( i = 0; i < 3; ++i ))
do
        offset=$(( 50*i ))
		case $i in
			0)
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=5 slurm100mForest_Line_RandomDrones.slurm
			;;
			1)
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=10 slurm100mForest_Line_RandomDrones.slurm
			;;
			2)
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=20 slurm100mForest_Line_RandomDrones.slurm
			;;
		esac
done

