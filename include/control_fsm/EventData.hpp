#ifndef EVENT_DATA_HPP
#define EVENT_DATA_HPP
/*
TODO
This class should contain all information a state might need to make a correct decision.
*/


//Request to go to a certain state
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
	GOTO, //GOTO XYZ
	LAND, 
	BLINDLAND, 
	TRACKGB, //Track/follow ground robot
	INTERGB, //Interact with ground robot (tap)
	ESTIMATORADJ //Estimator adjust
};

enum class EventType {
	NONE,
	REQUEST,
	ARMED,
	DISARMED,
	ERROR,
	POSREGAINED,
	POSLOST,
	TAKEOFFFINISHED
};

enum class CommandType {
    NONE, //Not part of a command
    LANDXY, //Request part of command to land at XY
    GOTOXYZ, //Reqeust part of going to specified XYZ
    LANDGB //Request part of interacting with ground robot
};

struct PositionGoalXYZ {
	bool valid = false;
	double x;
	double y;
	double z;
	PositionGoalXYZ(double posX, double posY, double posZ) : x(posX), y(posY), z(posZ), valid(true) {}
	PositionGoalXYZ() : valid(false) {}
};

class EventData {
public:
	RequestType request = RequestType::NONE; //No request as default
	EventType eventType = EventType::NONE; //No event as default
	PositionGoalXYZ positionTarget = PositionGoalXYZ(); //Invalid position as default
    CommandType commandType = CommandType::NONE; //No command as default
/*
Should contain all neccesary data for a state to make
neccesary decisions/transitions. Avoid large data copying if possible.
*/
};

#endif
