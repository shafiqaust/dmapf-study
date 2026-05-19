#!/bin/bash
# To stop: kill $(jobs -p)

prob="lak303d"
path="$HOME/catkin_ws/src/DMAPF/examples/${prob}"
solvers=20
areas=246

for seed in {101..110}; do
  echo "mkdir -p ${path}/seed${seed}"
  mkdir -p ${path}/seed${seed}

  if [ ! -f "${path}/${prob}.lp" ]; then
    rosrun dmapf map2lp -m ${path}/${prob}.map
  fi

  echo "cp ${path}/${prob}.lp ${path}/seed${seed}/${prob}.lp"
  cp ${path}/${prob}.lp ${path}/seed${seed}/${prob}.lp

  echo "rosrun dmapf gendiv -d -f ${path}/seed${seed}/${prob}.lp --bk-means -s ${solvers} -k ${areas} --seed ${seed} &> ${path}/seed${seed}/stats.txt &"
  rosrun dmapf gendiv -d -f ${path}/seed${seed}/${prob}.lp --bk-means -s ${solvers} -k ${areas} --seed ${seed} &> ${path}/seed${seed}/stats.txt &
done
wait
