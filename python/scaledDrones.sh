#!/bin/bash

for (( i = 0; i < 3; ++i ))
do
        offset=$(( 50*i ))
		sbatch --export=ALL,PORT_OFFSET=${offset},MOVEMENT_TYPE=${i} slurm100mForest_Random_ScaledDrones
done

