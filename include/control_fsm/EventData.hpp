#pragma once

/*
TODO
This class should contain all information a state might need to make a correct decision.
*/


//All the different states must have its own value
enum class RequestType {
	NONE,
	ABORT,
	BEGIN,
	END,
	PREFLIGHT,
	IDLE, 
	SHUTDOWN, 
	TAKEOFF, 
	BLINDHOVER, 
	POSHOLD, 
	GOTO, 
	LAND, 
	BLINDLAND, 
	TRACKGB, 
	INTERGB, 
	ESTIMATORADJ
};

enum class EventType {
	REQUEST,
	ARMED,
	DISARMED
}

class EventData {
public:
	RequestType request;
	EventType eventType
/*
Should contain all neccesary data for a state to make
neccesary decisions/transitions. Avoid large data copying if possible.
*/
};
