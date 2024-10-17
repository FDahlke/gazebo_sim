#!/bin/bash

for (( i = 0; i < 3; ++i ))
do
        offset=$(( 50*i ))
		case $i in
			0)
			echo running sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=5, MOVEMENT_TYPE=0 slurmDenseForest.slurm
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=5,MOVEMENT_TYPE=0 slurmDenseForest.slurm
			;;
			1)
			echo running sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=10, MOVEMENT_TYPE=0 slurmDenseForest.slurm
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=10,MOVEMENT_TYPE=0 slurmDenseForest.slurm
			;;
			2)
			echo running sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=20, MOVEMENT_TYPE=0 slurmDenseForest.slurm
			sbatch --export=ALL,PORT_OFFSET=${offset},NUM_DRONES=20,MOVEMENT_TYPE=0 slurmDenseForest.slurm
			;;
		esac
done

