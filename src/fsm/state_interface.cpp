#include "control/fsm/state_interface.hpp"

//This should mabye be moved to another file, but which one?

std::list<StateInterface *>* StateInterface::getAllStatesVector() {
    static std::list<StateInterface*> all_states_;
    return &all_states_;
}
