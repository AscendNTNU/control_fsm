#include "control/fsm/state_interface.hpp"

//This should maby be moved to another file, but which one?

std::vector<StateInterface *>* StateInterface::getAllStatesVector() {
    static std::vector<StateInterface*> all_states_;
    return &all_states_;
}
