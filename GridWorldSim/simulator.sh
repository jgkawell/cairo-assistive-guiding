#!/bin/bash
./GridWorldSim.pyw &
sleep 3
python3 humanSim.py &
sleep 3
python3 robotSim.py