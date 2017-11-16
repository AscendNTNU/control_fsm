#include "control/planner/node.hpp"


bool operator< (const Node &lhs, const Node &rhs){
	return lhs.f > rhs.f;
}

bool Node::isValid(){
    return (x>0 && x<FIELD_LENGTH && y>0 && y<FIELD_LENGTH);
}

void Node::updateF(float new_g){
    if(new_g < g){
        g = new_g;
    }
    f = g+h;
}