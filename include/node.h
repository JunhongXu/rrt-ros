#ifndef NODE_H
#define NODE_H
#include <Eigen/Dense>

using namespace Eigen;

class Node{
    public:
        // variables
        Vector2d node_pos;
        Node     *parent;

        // functions
        Node(Vector2d node_pos);
        Node();
        double calc_distance(Node &node);
};
#endif
