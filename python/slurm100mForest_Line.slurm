#!/bin/bash

#SBATCH --job-name=100Forest_line
#SBATCH --partition=all
#SBATCH --ntasks=1
#SBATCH --mem-per-cpu=2G
#SBATCH --cpus-per-task=32
#SBATCH --output=../logs/test_Line/log_%A.log    
#SBATCH --exclude=ant5,ant12 
#SBATCH --array=0-4

srun hostname
srun nproc

declare -i PORT_OFFSET=0

export GZ_DISCOVERY_MSG_PORT=$(( 20317 + PORT_OFFSET + 2*SLURM_ARRAY_TASK_ID ))
export GZ_DISCOVERY_SRV_PORT=$(( 20318 + PORT_OFFSET + 2*SLURM_ARRAY_TASK_ID ))
srun echo $GZ_DISCOVERY_MSG_PORT
srun echo $GZ_DISCOVERY_SRV_PORT    

srun apptainer exec ../apptainer/gazebo_sim-full.sif python py100Forest.py --numDrones 10 --popSize 10 --numGenerations 10 --maxRuns 50 --movementType 1
