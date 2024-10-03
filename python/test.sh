#!/bin/bash

for (( i = 0; i < 5; ++i ))
do
        offset=$(( 50*i ))
        sbatch --export=ALL,PORT_OFFSET=${offset} slurm100mForest_Line.slurm
done

