#!/bin/bash

#SBATCH --job-name=QuarterForestTest
#SBATCH --partition=all
#SBATCH --ntasks=1
#SBATCH --mem-per-cpu=8G
#SBATCH --cpus-per-task=4
#SBATCH --output=../logs/test_swarm/log_%A.log     

srun hostname
srun nproc
srun apptainer exec ../apptainer/gazebo_sim-full.sif python pyQuarterForest.py --numDrones 10 --popSize 10 --numGenerations 5 --maxRuns 1
