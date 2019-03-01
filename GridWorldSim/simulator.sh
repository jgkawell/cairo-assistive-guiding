#!/bin/bash
./GridWorldSim.pyw &
sleep 1
python3 humanSim.py &
sleep 1
python3 robotSim.py