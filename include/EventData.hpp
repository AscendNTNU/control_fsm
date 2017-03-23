#pragma once

//All the different states must have its own value
enum class RequestType {ABORT, BEGIN, END, PREFLIGHT, IDLE, SHUTDOWN, TAKEOFF, BLINDHOVER, POSHOLD, GOTO, LAND, BLINDLAND, TRACKGB, INTERGB, ESTIMATORADJ};


class EventData {
public:
	RequestType request;
/*
Should contain all neccesary data for a state to make
neccesary decisions/transitions. Avoid large data copying if possible.
*/
};
