#ifndef EVENT_DATA_HPP
#define EVENT_DATA_HPP
#include <functional>
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
	REQUEST, //Simple transition request
	COMMAND, //More complex command with multiple transitions
	ARMED, //Notify system is armed
	DISARMED, //Notify system is disarmed
	ERROR, //Notify there is an error
	POSREGAINED, //Notify position is regained
	POSLOST, //Notify position is lost
	TAKEOFFFINISHED //Notify takeoff is finished
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
	double yaw;
	PositionGoalXYZ(double posX, double posY, double posZ, double rotYaw) : x(posX), y(posY), z(posZ), yaw(rotYaw), valid(true) {}
	PositionGoalXYZ() : valid(false) {}
};

class EventData {
private:
	std::function<void()> _onComplete = []() {}; //Does nothing by default
	std::function<void(std::string)> _onError = [](std::string) {}; //Does nothing by default
public:
	RequestType request = RequestType::NONE; //No request as default
	EventType eventType = EventType::NONE; //No event as default
	PositionGoalXYZ positionTarget = PositionGoalXYZ(); //Invalid position as default
    CommandType commandType = CommandType::NONE; //No command as default

    void setOnCompleteCallback(std::function<void()> callback) { _onComplete = callback; }
    void setOnErrorCallback(std::function<void(std::string)> callback) { _onError = callback; }
    void finishEvent() const { _onComplete(); }
    void eventError(std::string errorMsg) { _onError(errorMsg); }

/*
Should contain all neccesary data for a state to make
neccesary decisions/transitions. Avoid large data copying if possible.
*/
};

#endif
