#!/usr/bin/python

# Input: times in format 1m23.456s
# Output: times in milliseconds

import sys

for i in range(1, len(sys.argv), 1):
    try:
        parts = sys.argv[i].split("m")
        ms = int(parts[0]) * 60000
        second_parts = parts[1].split(".")
        ms += int(second_parts[0]) * 1000
        sub_seconds = second_parts[1].split("s")[0]
        if len(sub_seconds) == 1:
            sub_seconds = sub_seconds + "00"
        elif len(sub_seconds) == 2:
            sub_seconds = sub_seconds + "0"
        ms += int(sub_seconds[0:3])
    except ValueError:
        print("ERR")
        continue
    print(ms)
