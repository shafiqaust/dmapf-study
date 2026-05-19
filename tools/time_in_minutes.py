#!/usr/bin/python

# Input: times in milliseconds
# Output: times in format 1m23.456s

import sys

for i in range(1, len(sys.argv), 1):
    try:
        ms = int(sys.argv[i])
    except ValueError:
        print("ERR")
        continue

    minutes = ms // 60000
    remainder = ms % 60000
    seconds = remainder // 1000
    remainder = remainder % 1000
    padding = ""
    if remainder < 10:
        padding = "00"
    elif remainder < 100:
        padding = "0"
    print(f"{minutes}m{seconds}.{padding}{remainder}s")
