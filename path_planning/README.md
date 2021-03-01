# Writing Path Planning Software
The path planning system is designed using a modular system of states. 
The submarine switches between different states, each of which performs a specialized task.

## States
All states are subclasses of the base_state:

https://github.com/Waterloo-Aquadrone/aquadrone2020/blob/dev/path_planning/src/path_planning/states/base_state.py

Functionality that allows a given state to interact with the submarine is provided in the initialize, process, and 
finalize functions of the state via the controls, sub_state, world_state, and sensors parameters. 
The functionality availible in each of these is documented in:

https://github.com/Waterloo-Aquadrone/aquadrone2020/blob/dev/path_planning/src/path_planning/ros_modules.py

See the waiting_state for a simple example of a state:
https://github.com/Waterloo-Aquadrone/aquadrone2020/blob/dev/path_planning/src/path_planning/states/waiting_state.py

See the barrel_roll state for a more complicated example:
https://github.com/Waterloo-Aquadrone/aquadrone2020/blob/dev/path_planning/src/path_planning/states/barrel_roll.py

## State Machines
State machines can be used to run several other states. These can be nested or chained however you want.
The existing state machines should cover the majority of basic functionality such as running states 
sequentially, in parallel, with a timer, and in a user-defined order (markov chain).
Nonetheless, you may wish to write your own state machine to bundle together a set of functionality.
For example, a single state machine could be created for finding and moving through a gate.


## Nodes
The highest level code is contained in the node files, which use the state machines and states to acheive a desired objective.
Each node should represent a single test that we might want to run on the submarine.

Check out the following file for an example of a node that uses several different states and state machines to perform a barrel roll:
https://github.com/Waterloo-Aquadrone/aquadrone2020/blob/dev/path_planning/nodes/barrel_roll_test.py
