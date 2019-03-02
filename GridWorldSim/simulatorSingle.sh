#!/bin/bash

./GridWorldSim.pyw &
sleep 1
# sys.argv: 0=name, 1=planner, 2=optimal
python3 humanSim.py True False & 
sleep 1
# sys.argv: 0=name, 1=abstract, 2=probabilistic_model
python3 robotSim.py True True &

wait

echo "All done"