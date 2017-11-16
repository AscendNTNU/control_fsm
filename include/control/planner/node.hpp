#ifndef NODE_HPP
#define NODE_HPP

#define FIELD_LENGTH 20


class Node{
private:
    float x;
    float y;

    float g;
    // heuristic, use diagonal distance
    float h;
    // Sum of g and h
    float f;

    float parent_x = -1;
    float parent_y = -1;
public:
    Node(){}
    Node(float x, float y, float g, float h):x(x), y(y), g(g), h(h){f = g+h;}

    float getX() const {return x;}
    float getY() const {return y;}
    float getF() const {return f;}
    float getG() const {return g;}
    float getParentX() const {return parent_x;}
    float getParentY() const {return parent_y;}

    void setG(float g) {this->g = g;}
    void setParentX(float parent_x){this->parent_x = parent_x;}
    void setParentY(float parent_y){this->parent_y = parent_y;}
    void updateF(float new_g);

    bool isValid();

    // Implemented only for the closed list priority queue
    friend bool operator< (const Node &lhs, const Node &rhs);
};

#endif // NODE_HPP