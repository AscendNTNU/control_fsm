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
	ESTIMATORADJ, //Estimator adjust
	MANUALFLIGHT
};

enum class EventType {
	NONE,
	REQUEST, //Simple transition request
	COMMAND,
	AUTONOMOUS, //Notify system is in autonomous mode (ARMED and OFFBOARD)
	MANUAL, //Notify system is in manual mode (DISARMED and/or not OFFBOARD)
	POSREGAINED, //Notify position is regained
	POSLOST, //Notify position is lost
	GROUNDDETECTED,
	OBSTACLECLOSING
};

enum class CommandType {
    NONE, //Not part of a command
    LANDXY, //Request part of command to land at XY
    GOTOXYZ, //Reqeust part of going to specified XYZ
    LANDGB //Request part of interacting with ground robot
};

///Defines a position goal
struct PositionGoalXYZ {
	bool valid = false;
	double x;
	double y;
	double z;
	double yaw;
	PositionGoalXYZ(double posX, double posY, double posZ, double rotYaw) : x(posX), y(posY), z(posZ), yaw(rotYaw), valid(true) {}
	PositionGoalXYZ() : valid(false) {}
};

class EventData;

///Contains all info about events
class EventData {
private:
	///Callback function when a CMD is completed
	std::function<void()> _onComplete = []() {}; //Does nothing by default
	///Callback function when a CMD fails.
	std::function<void(std::string)> _onError = [](std::string) {}; //Does nothing by default
public:
	RequestType request = RequestType::NONE; //No request as default
	EventType eventType = EventType::NONE; //No event as default
	PositionGoalXYZ positionGoal = PositionGoalXYZ(); //Invalid position as default
    CommandType commandType = CommandType::NONE; //No command as default

    ///Setter function for complete callback
    void setOnCompleteCallback(std::function<void()> callback) { _onComplete = callback; }
    ///Setter function for error callback
    void setOnErrorCallback(std::function<void(std::string)> callback) { _onError = callback; }
    ///Finishes a CMD (calls the _onComplete callback)
    void finishCMD() const { _onComplete(); }
    ///CMD error (calls _onError callback)
    void eventError(std::string errorMsg) const { _onError(errorMsg); }
    ///Checks if this event is a valid cmd type
    bool isValidCMD() const { return (eventType == EventType::COMMAND && commandType != CommandType::NONE); }
    ///Checks if this event is a valid request type
    bool isValidRequest() const { return (eventType == EventType::REQUEST && request != RequestType::NONE); }
};


///Wrapper class for LandXY CMD events
class LandXYCMDEvent : public EventData {
private:
	//Altitude to go to before landing
	const double _goToAltitude = 1.0f;
public:
	LandXYCMDEvent(double x, double y, double yaw) {
		positionGoal = PositionGoalXYZ(x, y, _goToAltitude, yaw);
		eventType = EventType::COMMAND;
		commandType = CommandType::LANDXY;
	}
};

///Wrapper class for GoToXYZ CMD events
class GoToXYZCMDEvent : public EventData {
public:
	GoToXYZCMDEvent(double x, double y, double z, double yaw) {
		positionGoal = PositionGoalXYZ(x,y,z,yaw);
		eventType = EventType::COMMAND;
		commandType = CommandType::GOTOXYZ;
	}
};

///Wrapper class for LandGB CMD events
class LandGBCMDEvent : public EventData {
public:
	//TODO Implement event type
};

///Wrapper class for requests events
class RequestEvent : public EventData {
public:
	RequestEvent(RequestType r) {
		eventType = EventType::REQUEST;
		request = r;
	}
};

#endif
