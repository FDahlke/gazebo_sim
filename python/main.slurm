#!/bin/bash

#SBATCH --job-name=test_swarm
#SBATCH --partition=all
#SBATCH --ntasks=1
#SBATCH --mem-per-cpu=4G
#SBATCH --cpus-per-task=4
#SBATCH --output=../logs/test_swarm/log_%A.log     

srun hostname
srun nproc
srun apptainer exec ../apptainer/gazebo_sim-full.sif python main.py --numDrones 10 --popSize 10 --numGenerations 10 --outputFile ../data/logs/FirstTest
