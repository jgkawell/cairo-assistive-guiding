#!/bin/bash
damages=()
damage=0

./GridWorldSim.pyw &
sleep 1
python3 humanSim.py &
sleep 2
python3 robotSim.py

echo All done