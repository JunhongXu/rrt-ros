#include "obstacle.h"
#include <cmath>

Obstacle::Obstacle(std::string type, double size, double height, VectorXd center):
    type(type), size(size), height(height), center(center){}


bool Obstacle::calc_collision_with_node(Node &node){
    // given a node ref, calculate whether the node is in collide with this obstacle
    VectorXd pos = node.node_pos.head(2);
    if(!type.compare("cyliner")){
        Node center_node(center);
        double dist = center_node.calc_distance(node);
        return dist <= size;
    }else{
        double x, y = pos[0], pos[1];
        double delta_x = fabs(x - center[0]);
        double delta_y = fabs(y - center[1]);
        return (delta_x <= size) && (delta_y <= size);
    }
}
