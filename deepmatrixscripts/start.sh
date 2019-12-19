#!/bin/bash

#Full Path to Processes
PROCESSES_TO_RUN=("/home/nvidia/MainReactor/build/rsfront" \
		  "/home/nvidia/morten/build/morten 10.4.88.20 554" \
		  "/home/nvidia/targetingcomputer/build/targetingcomputer 10.4.88.20 5802")

for i in ${PROCESSES_TO_RUN[@]}; do 
	${i%/*}/./${i##*/} & #> ${i}.log 2>&1 &
	# ${i%/*} -> Get folder name until the /
	# ${i##*/} -> Get the filename after the / 
done

wait
