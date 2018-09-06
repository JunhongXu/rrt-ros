#ifndef OBS_H
#define OBS_H
#include <string>
#include <Eigen/Dense>

using namespace Eigen;

class Obstacle{
    public:
        // CUBE or CYLINDER
        std::string type;
        // length/diameter of the CUBE or CYLINDER
        double size;
        double height;
        VectorXd center;

        Obstacle(std::string type, double size, double height, VectorXd center);
};
#endif
