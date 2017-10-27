#ifndef EVENT_DATA_HPP
#define EVENT_DATA_HPP
#include <functional>
/*
This class should contain all information a state might need to make a correct decision.
*/


///Request types - "ask" fsm to do transition (or abort)
enum class RequestType {
    NONE,
    ABORT,
    BEGIN,
    END,
    PREFLIGHT,//newcomment
    IDLE, 
    TAKEOFF, 
    BLINDHOVER, 
    POSHOLD, 
    GOTO, //GOTO XYZ
    LAND, 
    TRACKGB, //Track/follow ground robot
    INTERGB, //Interact with ground robot (tap)
    MANUALFLIGHT
};

///Event types - what type of event is it?
enum class EventType {
    NONE,
    REQUEST, //Simple transition request
    COMMAND,
    AUTONOMOUS, //Notify system is in autonomous mode (ARMED and OFFBOARD)
    MANUAL, //Notify system is in manual mode (DISARMED and/or not OFFBOARD)
    POSLOST, //Notify position is lost
    GROUNDDETECTED
};

///Command types - what command is it?
enum class CommandType {
    NONE, //Not part of a command
    LANDXY, //Request part of command to land at XY
    GOTOXYZ, //Reqeust part of going to specified XYZ
    //LANDGB //Request part of interacting with ground robot
};

///Defines a position goal
struct PositionGoalXYZ {
    bool valid = false;
    double x;
    double y;
    double z;
    PositionGoalXYZ(double posX, double posY, double posZ) : x(posX), y(posY), z(posZ), valid(true) {}
    PositionGoalXYZ() : valid(false) {}
};

class EventData;

///Contains all info about events
class EventData {
private:
    ///Callback function when a CMD is completed
    std::function<void()> onComplete_ = []() {}; //Does nothing by default
    ///Callback function when a CMD fails.
    std::function<void(std::string)> onError_ = [](std::string) {}; //Does nothing by default
    //Callback function for sendig feedback during cmd execution
    std::function<void(std::string)> onFeedback_ = [](std::string){};
public:

    ///If event is a request - what type?
    RequestType request = RequestType::NONE; //No request as default
    ///What type of event is it?
    EventType eventType = EventType::NONE; //No event as default
    ///Whats the target (if needed by event)
    PositionGoalXYZ positionGoal = PositionGoalXYZ(); //Invalid position as default
    ///If event is a command, what type?
    CommandType commandType = CommandType::NONE; //No command as default

    ///Setter function for complete callback
    void setOnCompleteCallback(std::function<void()> callback) { onComplete_ = callback; }
    ///Setter function for error callback
    void setOnErrorCallback(std::function<void(std::string)> callback) { onError_ = callback; }
    ///Setter function for feedback callback
    void setOnFeedbackCallback(std::function<void(std::string)> callback) {onFeedback_ = callback; }
    ///Finishes a CMD (calls the _onComplete callback)
    void finishCMD() const { onComplete_(); }
    ///CMD error (calls _onError callback)
    void eventError(std::string errorMsg) const { onError_(errorMsg); }
    ///Sends CMD feedback via _onFeedback callback
    void sendFeedback(std::string msg) const { onFeedback_(msg); }
    ///Checks if this event is a valid cmd type
    bool isValidCMD() const { return (eventType == EventType::COMMAND && commandType != CommandType::NONE); }
    ///Checks if this event is a valid request type
    bool isValidRequest() const { return (eventType == EventType::REQUEST && request != RequestType::NONE); }
};


///Wrapper class for LandXY CMD events
class LandXYCMDEvent : public EventData {
private:
    //Altitude to go to before landing
    const double goToAltitude_ = 1.0f;
public:
    LandXYCMDEvent(double x, double y) {
        positionGoal = PositionGoalXYZ(x, y, goToAltitude_);
        eventType = EventType::COMMAND;
        commandType = CommandType::LANDXY;
    }
};

///Wrapper class for GoToXYZ CMD events
class GoToXYZCMDEvent : public EventData {
public:
    GoToXYZCMDEvent(double x, double y, double z) {
        positionGoal = PositionGoalXYZ(x,y,z);
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
