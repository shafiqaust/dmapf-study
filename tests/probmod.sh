#!/bin/bash

prob="lak303d"
type="random" # random or even
path="$HOME/catkin_ws/src/DMAPF/examples/${prob}"
solvers=20
robots=600
scene=1

for seed in {101..110}; do
  command=""

  for (( d=1; d<=$solvers; d++ )); do
    command+="${path}/seed${seed}/d${d}.lp "
  done

  command+="${path}/scen-${type}/${prob}-${type}-${scene}.scen "
  command+="${robots}"

  echo "rosrun dmapf probmod ${command}"
  rosrun dmapf probmod ${command}
done
