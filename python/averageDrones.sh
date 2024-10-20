#!/bin/bash

for (( i = 0; i < 3; ++i ))
do
        offset=$(( 50*i ))
		case $i in
			0)
			echo running sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=10 slurmAverageForest.slurm
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=10,MOVEMENT_TYPE=0 slurmSparseForest.slurm
			;;
			1)
			echo running sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=10 slurmAverageForest.slurm
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=10,MOVEMENT_TYPE=0 slurmAverageForest.slurm
			;;
			2)
			echo running sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=5 slurmAverageForest.slurm
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=5,MOVEMENT_TYPE=0 slurmAverageForest.slurm
			;;
		esac
done

