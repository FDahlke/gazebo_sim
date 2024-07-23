#!/bin/bash

#SBATCH --job-name=example_swarm
#SBATCH --partition=all
#SBATCH --ntasks=1
#SBATCH --mem-per-cpu=4G
#SBATCH --cpus-per-task=16
#SBATCH --output=../logs/example_swarm/log_%A.log     

srun hostname
srun nproc
srun apptainer exec ../apptainer/gazebo_sim-full.sif python cluster_swarm.py

