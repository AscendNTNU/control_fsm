# Control finite state machine (FSM)

[![Build Status](http://build.ascendntnu.no/buildStatus/icon?job=control_fsm)](http://build.ascendntnu.no/job/control_fsm/)

Finite state machine design pattern to avoid messy (buggy) code when the number of states increases. 

It's a ros package, build as usual with catkin_make.
The control_fsm_test node is only for testing the FSM logic


Some notes:

- An event is a external request to do a transition, passed to the FSM.

- Each state handles all legal transition from itself. 


