#include "node.h"
#include <iostream>

Node::Node(Vector2d node_pos){
    this->node_pos = node_pos;
    this->parent = nullptr;
}

Node::Node(){
    this->node_pos = Vector2d();
    this->parent = nullptr;
}

double Node::calc_distance(Node &node){
    Vector2d other_node_pos = node.node_pos;
    Vector2d distance = other_node_pos - this -> node_pos;
    distance = distance.array().pow(2);
    double scalar_distance = distance.sum();
    scalar_distance = sqrt(scalar_distance);
    return scalar_distance;
}
