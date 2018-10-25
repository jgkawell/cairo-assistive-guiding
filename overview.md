## Project overview

Input:
- The ID of the worker's current state (observation)
- Eventually will be a probability distribution over all possible states


Output:
- The ID of the assistive action to take (action)


Internal:
- The instruction set (state space): this will be stored in the CCHTN
- List of potential assistive actions (action space)
  - Contained within CCHTN
  - The CCHTN ought to limit the action space by removing the currently unaccessible materials/actions


Process:
- MPD that maps observations to actions
- Attempts to push human into the optimal path by manipulating supportive behaviors
- The optimal path is given to the robot (we can take this out later)
- Learns a policy based on how effective each assistive action is in pushing the human toward a certain state




## Overview of Maze Simulation:

Input:
- Coordinates of current location

Output:
- ID of sentence to try and convince the user to move a certain direction

Internal:
- The potential states that the user could take
  - This space should be reduced based off of the current maze setup
- The potential sentences (actions) to take


Reponse options:
- Good for binary:
  - Don't go x way
  - Going x way is a deadend
- Good for tertiary:
  - Going x way will be the shortest
  - I think x way is the best 
