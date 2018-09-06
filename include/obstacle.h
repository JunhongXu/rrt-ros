#ifndef OBS_H
#define OBS_H
#include <string>
#include <Eigen/Dense>
#include <node.h>
using namespace Eigen;

class Obstacle{
    public:
        // CUBE or CYLINDER
        const std::string type;
        // length/diameter of the CUBE or CYLINDER
        const double size;
        const double height;
        const VectorXd center;

        Obstacle(std::string type, double size, double height, VectorXd center);

        bool calc_collision_with_node(Node &node);

};
#endif
