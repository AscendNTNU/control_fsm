# Control finite state machine (FSM)

[![Build Status](http://build.ascendntnu.no/buildStatus/icon?job=control_fsm)](http://build.ascendntnu.no/job/control_fsm/)

It's a ros package, build as usual with catkin_make. It requires that you have ascend_msgs in your catking workspace

**FSM depends on:**
- Mavros for position, velocity and drone state.

**Some notes:**
- The control node will automatically transition to preflight state when fsm is ready.
    - In preflight it will wait for an AUTONOMOUS event (OFFBOARD mode and ARMED) and then transition to idle
    - An alternative is to manually request manual flight mode to fly around manually before transitioning to positionhold when switching to OFFBOARD
- Each state handles all legal transition from itself. (ONLY exception is from begin state to preflight which is forced by fsm)
- CMDs are "special" events that can be stored by a state, and allows autotransition to next state. CMDs
can therefore "ripple" through the fsm to reach the target state. See RequestsVsCommands.md
- Do not send manual debug requests to fsm in normal operation.
    - The only purpose of a manual request is debugging, and internal transitions! Be careful!
- Tracking and interacting with groundrobots is not yet implemented
- Altitude restrictions is implemented. Drone will not attempt to fly below min_in_air_altitude!

**Coordinate frames: **
The statemachine assumes these frames are set correctly:
- map = Global position, not allowed to drift over time
- odom = local drone position, allowed to drift over time (used by drone, mavros)
- base_link = Body frame fixed to drone

**Startup procedure**
1. Launch FSM from launchfile
```
roslaunch control_fsm control_fsm.launch
```
2. On startup FSM will initialize all states and set up necessary ros publishers, subscribers and services.
3. FSM will then wait for all ros data from other nodes to be available
    - Some of the data streams that is not needed for flight can be skipped for testing if require_all_streams is set to false
4. The actionserver will then become active, but not accept actions until enabled by user (via a ros service)
5. Preflight is finished. The FSM will then transition to preflight mode automaticly and wait for an AUTONOMOUS event (drone is in OFFBOARD mode and ARMED). All other events are ignored
6. The node will loop at 30Hz and publish setpoints to mavros

**Design pattern inspiration:**
http://gameprogrammingpatterns.com/state.html

**CMD rules:**
- ABORT request should be sent before new command if old command is not finished!
- Remember to terminate active commands if they're not passed on in the states.
- Use CMD wrapper classes to generate CMDs - to make sure the events contains all neccesary information.
- Remember to setup CMD callback functions. 

**For developers:**
- It shouldn't be possible to send a new command if another is running. (ABORT first)
- Remeber to check if there is any active command before transitioning
    - If there is: Terminate, or pass the CMD on (depending on current state)
- Each state is responsible for delivering valid setpoints (through getSetpointPtr method)
- Each state should be as independent as possible.
- Use Config class (static) for loading ros parameters
- Be sure to update state diagram when adding / removing states

**Doxygen support is added.**

*Requires doxygen*

Run:
```
doxygen Doxyfile
```
to generate documentation (html and latex)

**ROS Params:**

```
Name: "dest_reached_margin", Description: Amount of error we can allow in position for a destination target
```
```
Name: "blind_hover_altitude", Description: Altitude used if position is lost (never really used)
```
```
Name: "takeoff_altitude", Description: Altitude setpoint for takeoff
```
```
Name: "altitude_reached_margin", Description: Allowed altitude error for takeoff
```
```
Name: "setp_reached_margin", Description: How close we have to be to a setpoint before switching (in a path plan)
```
```
Name: "fsm_error_topic", Description: Topic for publishing fsm error messages
```
```
Name: "fsm_warn_topic", Description: Topic for publishing fsm warn messages
```
```
Name: "fsm_state_changed_topic", Description: Topic for publishing when state changes
```
```
Name: "status_msg_buffer_size", Description: Buffersize for status msgs from FSM
```
```
Name: "goto_hold_dest_time", Description: How long will the drone wait at destination before transitioning? *
```
```
Name: "safe_hover_alt", Description: Altitude where drone is safe from all obstacles
```
```
Name: "obstacle_too_close_dist", Description: Drone "safezone" **
```
```
Name: "lidar_topic", Description: Topic for recieving obstacle detections from lidar
```
```
Name: "yaw_reached_margin", Description: Max allowed error for yaw setpoint
```
```
Name: "no_yaw_correct_dist", Description: Not setting new yaw in gotostate if target is too close
```
```
Name: "require_all_streams", Description: Require that all datastreams (from other nodes) is available before continuing (DEBUG feature) 
```
```
Name: "message_timeout", Description: When is data from other nodes considered old 
```
```
Name: "local_velocity_topic", Description: Mavros velocity topic, MsgType: geometry_msgs/TwistStamped
```
```
Name: "local_position_topic", Description: Mavros position topic, MsgType: geometry_msgs/PoseStamped
```
```
Name: "mavros_state_topic", Description: Mavros state topic (info about OFFBOARD and ARMED) 
```
```
Name: "land_detector_topic", Description: Topic for knowing if drone is on ground, MsgType: ascend_msgs/BoolStamped 
```
```
Name: "land_xy_goto_altitude", Description: Decides what altitude to use before landing on landxy cmd
```
```
Name: "min_in_air_altitude", Description: Minimum altitude drone is allowed to fly (except when landing) 
```
```
Name: "velocity_reached_margin", Description: Allowed deviation from target velocity
```
```
Name: "global_frame_id", Description: Frame id for global frame, Default: map
```
```
Name: "local_frame_id", Description: Frame id for local frame, Default: odom
```

*Drone have to wait to allow the drone to slow down before doing a transition

**Drone will automaticly go to safe hover altitude if an obstacle is too close

