# Control finite state machine (FSM)
Finite state machine design pattern to avoid messy (buggy) code when the number of states increases. 

It's a ros package, build as usual with catkin_make.
The control_fsm_node is only for testing the FSM logic


Some notes:

- An event is a external request to do a transition, passed to the FSM.

- All events will result in a transition. If no legal transition is found, the node is transitioning too itself. In practice, this means the stateBegin function will be called after each event, even if there hasn't been a transition.  

- Each state handles all legal transition from itself. 
