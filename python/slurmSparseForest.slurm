#!/bin/bash

#SBATCH --job-name=100Forest_rnd
#SBATCH --partition=all
#SBATCH --ntasks=1
#SBATCH --mem-per-cpu=2G
#SBATCH --cpus-per-task=32
#SBATCH --output=../logs/test_20DroneStatic/log_%A_%a.log    
#SBATCH --exclude=ant5,ant12 
#SBATCH --array=0-4

srun hostname
srun nproc

export GZ_DISCOVERY_MSG_PORT=$(( 20317 + PORT_OFFSET + 2*SLURM_ARRAY_TASK_ID ))
export GZ_DISCOVERY_SRV_PORT=$(( 20318 + PORT_OFFSET + 2*SLURM_ARRAY_TASK_ID ))
srun echo $GZ_DISCOVERY_MSG_PORT
srun echo $GZ_DISCOVERY_SRV_PORT    

echo srun apptainer exec ../apptainer/gazebo_sim-full.sif python py100Forest.py --numDrones $((NUM_DRONES)) --popSize 10 --numGenerations 5 --maxRuns 50 --movementType $((MOVEMENT_TYPE)) --worldFile world100ForestSparse
srun apptainer exec ../apptainer/gazebo_sim-full.sif python py100Forest.py --numDrones $((NUM_DRONES)) --popSize 10 --numGenerations 5 --maxRuns 50 --movementType $((MOVEMENT_TYPE)) --worldFile world100ForestSparse
