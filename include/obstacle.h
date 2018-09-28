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
        Vector2d upper_left;
        Vector2d upper_right;
        Vector2d lower_left;
        Vector2d lower_right;
        std::vector<std::vector<Vector2d>> lines;
        Obstacle(std::string type, double size, double height, VectorXd center);

        bool collision_with_rect(Node &node_one, Node &node_two);

        bool collision_with_line(std::vector<Vector2d> line_one, std::vector<Vector2d> line_two);

        std::vector<double> calc_coeff(std::vector<Vector2d> line);

};
#endif
