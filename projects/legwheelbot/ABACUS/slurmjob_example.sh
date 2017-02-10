#!/usr/bin/env bash
#
#SBATCH --account sdubats_slim      # account
#SBATCH --nodes 50                  # number of nodes
#SBATCH --ntasks-per-node 3         # number of MPI tasks per node
#SBATCH --time 10:00:00             # max time (HH:MM:SS)

# SEE THE ABACUS DOCUMENTATION FOR DETAILS ON HOW TO SCHEDULE JOBS

echo Running on "$(hostname)"
echo Available nodes: "$SLURM_NODELIST"
echo Slurm_submit_dir: "$SLURM_SUBMIT_DIR"
echo Start time: "$(date)"

# Load any required modules
module purge
module add gcc/4.8-c7 openmpi/1.10.2

# Setup a proper environment
export LD_LIBRARY_PATH=~/lpz_dependencies/usr/lib64:~/workspace/pmanoonpong-lpzrobots-fork/opende/ode/src/.libs
export QMAKESPEC=/lib64/qt4/mkspecs/linux-g++
export CPATH=~/lpz_dependencies/usr/include

# Example of how the legwheelbot application can be launched as an mpi program
# On abacus srun is used instead of mpirun
cd /home/almeh/workspace/pmanoonpong-gorobots-fork/projects/legwheelbot
srun sim -simtime 1 -terrains ice-rock-concrete-sand-swamp-rubber -ngen 1000 -popsize 150 -nographics

echo Done.
