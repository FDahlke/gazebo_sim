#!/bin/bash

#SBATCH --job-name=100Forest
#SBATCH --partition=all
#SBATCH --ntasks=1
#SBATCH --mem-per-cpu=2G
#SBATCH --cpus-per-task=32
#SBATCH --output=../logs/test_swarm/log_%A.log     

srun hostname
srun nproc
srun apptainer exec ../apptainer/gazebo_sim-full.sif python py100Forest.py --numDrones 10 --popSize 10 --numGenerations 10 --maxRuns 50
