#!/bin/bash
# https://unix.stackexchange.com/questions/55913/whats-the-easiest-way-to-find-an-unused-local-port/423052#423052

set -x
envfile="$1"
port="$(comm -23 <(seq 49152 65535 | sort) <(ss -Htan | awk '{print $4}' | cut -d':' -f2 | sort -u) | shuf | head -n 1)"
node="$(hostname)"
export ROS_MASTER_URI="http://${node}:${port}/"
echo "$ROS_MASTER_URI" > "$envfile"
roscore -p $port  # using rosmaster may be better than roscore (less logging?)
