#!/usr/bin/python

# Input: files containing outputs from DMAPF
# Output: statistics, adjustable in the program

import re
import sys

for i in range(1, len(sys.argv), 1):
    file = open(sys.argv[i], mode='r')
    content = file.read()
    file.close()

    num_relaxes = content.count("relaxes")

    abstract = re.findall(r'Abstract\s*:\s*\w+', content)
    if abstract:
        abstract = abstract[0].split()
        if len(abstract) == 3:
            abstract = abstract[2]
        else:
            abstract = "ERR"
    else:
        abstract = "ERR"

    solver = re.findall(r'Solver\s*:\s*\w+', content)
    if solver:
        solver = solver[0].split()
        if len(solver) == 3:
            solver = solver[2]
        else:
            solver = "ERR"
    else:
        solver = "ERR"

    time = re.findall(r'Time\s*:\s*[\d|m|s|\.]+', content)
    if time:
        time = time[0].split()
        if len(time) == 3:
            time = time[2]
        else:
            time = "ERR"
    else:
        time = "ERR"

    makespan = re.findall(r'Makespan\s*:\s*\d+', content)
    if makespan:
        makespan = makespan[0].split()
        if len(makespan) == 3:
            makespan = makespan[2]
        else:
            makespan = "ERR"
    else:
        makespan = "ERR"

    soc = re.findall(r'SoC\s*:\s*\d+', content)
    if soc:
        soc = soc[0].split()
        if len(soc) == 3:
            soc = soc[2]
        else:
            soc = "ERR"
    else:
        soc = "ERR"

    print(f"{sys.argv[i]}, {abstract}, {solver}, {time}, {makespan}, {soc}, {num_relaxes}")
