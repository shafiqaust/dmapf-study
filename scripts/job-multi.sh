#!/bin/bash

#SBATCH --job-name=x40_y40_n1280_r256_m
#SBATCH --output=multi-%j.out
#SBATCH --ntasks=26
#SBATCH --cpus-per-task=2
#SBATCH --mem-per-cpu=1G
#SBATCH --partition=normal
#SBATCH --time=0-00:15:00

module load conda
conda activate ros-noetic
source $HOME/catkin_ws/devel/setup.bash
set -x

srun --nodes=1 --ntasks=1 --cpus-per-task=$SLURM_CPUS_PER_TASK ./job-roscore.sh "roscore-$SLURM_JOB_ID.out" &
sleep 10s
export ROS_MASTER_URI="$(cat roscore-$SLURM_JOB_ID.out)"
if [[ -z $ROS_MASTER_URI ]]; then
  echo "ROS_MASTER_URI has not been set" >&2
  exit 1
fi

path=$(rospack find dmapf)
if [[ -z $path ]]; then
  echo "Cannot find project dmapf" >&2
  exit 2
fi
rosparam set abstract $path/src/abstract.lp
rosparam set migrate $path/src/migrate.lp
rosparam set movement $path/src/movement.lp
rosparam set x40_y40_n1280_r256_m/problem $path/examples/x40_y40_n1280_r256/x40_y40_n1280_r256.lp
rosparam set x40_y40_n1280_r256_m/answer $path/examples/x40_y40_n1280_r256/answer.lp
rosparam set x40_y40_n1280_r256_m/links $path/examples/x40_y40_n1280_r256/links.lp
rosparam set x40_y40_n1280_r256_m/areas 25
rosparam set x40_y40_n1280_r256_m/solvers $((SLURM_NTASKS - 1))

for (( i=1 ; i<$SLURM_NTASKS; i++ )); do
  srun --nodes=1 --ntasks=1 --cpus-per-task=$SLURM_CPUS_PER_TASK rosrun dmapf solver __ns:=x40_y40_n1280_r256_m __name:=s$i _domain:=$path/examples/x40_y40_n1280_r256/d$i.lp &
done
wait

# roscore will not terminate automatically - may need to fix this
