#!/bin/bash

home="$HOME/catkin_ws/src/DMAPF"
type="random" # random or even scenario
timeout=300   # in seconds
solvers=20

# Available systems: DMAPF, CBSH2, EECBS, PBS
system="DMAPF"
subsys="ASP"    # Either ASP, CBSH2, EECBS, or PBS

suboptimality=5.0   # For EECBS

for prob in "lak303d"; do
  path="${home}/examples/${prob}"

  if [ ! -d ${path} ]; then
    echo "${path} does not exist"
    continue
  fi

  for robots in 100; do
    for scene in {1..10}; do
      echo ${path}
      echo "Robots = ${robots}"
      echo "Scene = ${prob}-${type}-${scene}.scen"

      if [ "${system}" = "DMAPF" ]; then
        for seed in {101..110}; do
          echo "Seed = ${seed}"

          # Modify the scenario
          command=""

          for (( d=1; d<=$solvers; d++ )); do
            command+="${path}/seed${seed}/d${d}.lp "
          done

          command+="${path}/scen-${type}/${prob}-${type}-${scene}.scen "
          command+="${robots}"

          rosrun dmapf probmod ${command}
          sleep 1s

          # Run DMAPF
          echo "timeout ${timeout} roslaunch dmapf ${prob}-seed${seed}.launch &> ${path}/seed${seed}/${system}-${subsys}-r${robots}-${type}${scene}.txt"
          timeout ${timeout} roslaunch dmapf ${prob}-seed${seed}.launch &> ${path}/seed${seed}/${system}-${subsys}-r${robots}-${type}${scene}.txt

          rosclean purge -y

          echo "sleep 10s"
          sleep 10s
        done
      elif [ "${system}" = "CBSH2" ]; then
        ${home}/solvers/CBSH2-RTC/cbs -m ${path}/${prob}.map -a ${path}/scen-${type}/${prob}-${type}-${scene}.scen \
                                      -o ${path}/${system}-r${robots}.csv --outputPaths=${path}/${system}-r${robots}-${type}${scene}.txt \
                                      -k ${robots} -t ${timeout}
        sleep 1s
      elif [ "${system}" = "EECBS" ]; then
        ${home}/solvers/EECBS/eecbs -m ${path}/${prob}.map -a ${path}/scen-${type}/${prob}-${type}-${scene}.scen \
                                    -o ${path}/${system}-r${robots}.csv --outputPaths=${path}/${system}-r${robots}-${type}${scene}.txt \
                                    -k ${robots} -t ${timeout} --sipp=1 --suboptimality=${suboptimality}
        sleep 1s
      elif [ "${system}" = "PBS" ]; then
        ${home}/solvers/PBS/pbs -m ${path}/${prob}.map -a ${path}/scen-${type}/${prob}-${type}-${scene}.scen \
                                -o ${path}/${system}-r${robots}.csv --outputPaths=${path}/${system}-r${robots}-${type}${scene}.txt \
                                -k ${robots} -t ${timeout} --sipp=1
        sleep 1s
      else
        echo "Unknow system: $system"
        exit 1
      fi
    done
  done
done