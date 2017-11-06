#ifndef CONTROL_FSM_HPP
#define CONTROL_FSM_HPP

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <functional>

#include "state_interface.hpp"
#include "begin_state.hpp"
#include "preflight_state.hpp"
#include "idle_state.hpp"
#include "takeoff_state.hpp"
#include "blind_hover_state.hpp"
#include "position_hold_state.hpp"
#include "track_gb_state.hpp"
#include "interact_gb_state.hpp"
#include "go_to_state.hpp"
#include "land_state.hpp"
#include "manual_flight_state.hpp"

#include "control/tools/land_detector.hpp"

#include "control/tools/control_pose.hpp"

///Main FSM logic
class ControlFSM {
private:
    ///Shared instance pointer
    static std::shared_ptr<ControlFSM> shared_instance_p_;

    //Add state classes as friend classes here - allowing them to use transitionTo.
    friend class BeginState;
    friend class PreFlightState;
    friend class IdleState;
    friend class TakeoffState;
    friend class BlindHoverState;
    friend class PositionHoldState;
    friend class TrackGBState;
    friend class InteractGBState;
    friend class GoToState;
    friend class LandState;
    friend class ManualFlightState;
    
    //Static instances of the different states
    //Also add them to allStates_ vector in constructor
    static BeginState BEGIN_STATE;
    static PreFlightState PREFLIGHT_STATE;
    static IdleState IDLE_STATE;
    static TakeoffState TAKEOFF_STATE;
    static BlindHoverState BLIND_HOVER_STATE;
    static PositionHoldState POSITION_HOLD_STATE;
    static TrackGBState TRACK_GB_STATE;
    static InteractGBState INTERACT_GB_STATE;
    static GoToState GO_TO_STATE;
    static LandState LAND_STATE;
    static ManualFlightState MANUAL_FLIGHT_STATE;

    /**
     * @brief Holds a pointer to current running state
     * @details Struct "vault" explanation:
     *    The struct (with instance _stateHolder) keeps the _pCurrentState private. 
     *    The struct friends the FSM class so the FSM class can access the pointer. 
     *    Even though other classes or function might have access to FSM private variables through friend,
     *    they still wont have access to the pointer.
     */
    struct {
        friend class ControlFSM;
    private:
        StateInterface* current_state_p_ = nullptr; //This need to be set to a start state in constructor
    } state_vault_;

    ///Current drone position
    struct {
        //Not all states needs direct access to position and flags
        friend class ControlFSM;
        friend class EstimateAdjustState;
    private:
        geometry_msgs::PoseStamped position;
        bool is_set = false;
        bool valid_xy = true; //Assumes XY is valid if not set otherwise
    } drone_position_;

    ///Struct holding information about drones state
    struct {
        bool is_offboard = false;
        bool is_armed = false;
        bool is_preflight_completed = false;
    } drone_state_;

    ///Has FSM been initiated?
    bool states_is_ready_ = false;

    ///Callback when a transition is made
    std::function<void()> on_state_changed_ = [](){};

    ///Constructor sets default/starting state
    ControlFSM();

    ///Copy constructor deleted
    ControlFSM(const ControlFSM&) = delete;
    ///Assignment operator deleted
    ControlFSM& operator=(const ControlFSM&) = delete;

    ///Shared nodehandle for all states
    ros::NodeHandle node_handler_;
    ///Struct holding all shared ControlFSM ros subscribers
    struct {
        friend class ControlFSM;
    private:
        ros::Subscriber mavros_state_changed_sub;
    } subscribers_;

    ///Drones pose
    std::shared_ptr<control::Pose> drone_pose_p = control::Pose::getSharedPosePtr();

    ///Callback for mavros state changed
    void mavrosStateChangedCB(const mavros_msgs::State& state);

    ///Initializes all states
    void initStates();


protected:
    /**
     * @brief Changes the current running state
     * @details Allows the current running state to change the current state pointer
     * @param state Which state instance to transition to0
     * @param caller_p Which state that requests the transition
     * @param event Which event triggered the transition request
     */
    void transitionTo(StateInterface& state, StateInterface* caller_p, const EventData& event);
    
public:

    /**Get shared instance pointer - singleton patter
     * @throw control::ROSNotInitialized
     * @throw std::bad_alloc
     * @return shared_ptr to shared instance - singleton
     */
    static std::shared_ptr<ControlFSM> getSharedInstancePtr();

    ///Destructor not used to anything specific.
    ~ControlFSM() {}

    ///Get pointer to the current running state
    StateInterface* getState() { return state_vault_.current_state_p_; }
    
    /**
     * @brief Handles incoming (external) events
     * @details Events are sent to current running state
     * @param event Information about the external event
     */
    void handleEvent(const EventData& event);
    
    ///Loops the current running state
    void loopCurrentState(void);

    
    ///Returns setpoint from current state
    const mavros_msgs::PositionTarget* getSetpoint() { return getState()->getSetpoint(); }

    ///Sets new callback function for onStateChanged
    void setOnStateChangedCB(std::function<void()> cb) { on_state_changed_ = cb; }

    ///Checks if all states are ready
    bool isReady();

    ///Transition to preflight from begin state
    void startPreflight();

    ///Handles loss of offboard mode
    void handleManual();
};

#endif

