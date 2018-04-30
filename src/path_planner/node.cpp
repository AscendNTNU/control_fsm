#include "control/planner/node.hpp"


bool operator< (const Node &lhs, const Node &rhs){
    return lhs.f > rhs.f;
}

void Node::updateF(float new_g){
    g = new_g;
    f = g+h;
}
