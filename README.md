# Control finite state machine (FSM)

[![Build Status](http://build.ascendntnu.no/buildStatus/icon?job=control_fsm)](http://build.ascendntnu.no/job/control_fsm/)

Finite state machine design pattern to avoid messy (buggy) code when the number of states increases. 

It's a ros package, build as usual with catkin_make.
The control_fsm_test node is only for testing the FSM logic

Some notes:

- An event is a external request to do a transition, passed to the FSM.

- Each state handles all legal transition from itself. 

Design pattern inspiration:
http://gameprogrammingpatterns.com/state.html

CMD rules:
- ABORT request should be sent before new command if old command is not finished!
- Remember to terminate active commands if they're not passed on in the states.
- Use CMD wrapper classes to generate CMDs - to make sure the events contains all neccesary information.
- Remember to setup CMD callback functions. 

For developers:
- It shouldn't be possible to send a new command if another is running. (ABORT first)
- Remeber to check if there is any active command before transitioning
	- If there is: Terminate, or pass the CMD on (depending on current state)
- Each state is responsible for delivering valid setpoints (through getSetpoint method)

Doxygen support is added.
Run:
```
doxygen Doxyfile
```
to generate documentation



