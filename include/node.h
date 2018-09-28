#ifndef NODE_H
#define NODE_H
#include <Eigen/Dense>

using namespace Eigen;

class Node{
    public:
        // variables
        Vector3d node_pos;
        Node     *parent = nullptr;

        // functions
        Node(Vector3d node_pos);
        Node();
        double calc_distance(Node &node);
};
#endif
