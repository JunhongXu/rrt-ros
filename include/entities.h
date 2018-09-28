#ifndef ENTITIES_H
#define ENTITIES_H
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include <memory>
namespace entities
{
    struct Robot
    {
        unsigned int n_dof;
        // width and length
        std::vector<double> dim;
        // robot orientation angle
        const double init_theta;
        double theta;
        // assume the robot is rectangular, this is the center of the robot at the initial position
        const Eigen::Vector2d init_pos;
        Eigen::Vector2d position;
        // each element represents a point in the configuration space, now it is (x, y, theta)
        Eigen::Matrix<double, 3, 1> cspace_repr;
        // rotation matrix for the robot
        Eigen::Rotation2D<double> rotation_matrix;
        // points that represent the square robot
        std::vector<std::shared_ptr<Eigen::Vector2d>> points;
        std::vector<Eigen::Vector2d> init_points;

        Robot(unsigned int n_dof, std::vector<double> dim, double init_theta, Eigen::Vector2d init_pos,
                double extra_space):
            n_dof(n_dof), dim(dim), init_theta(init_theta), position(init_pos), init_pos(init_pos)
        {
            // boost detection needs some extra space
            std::shared_ptr<Eigen::Vector2d> upper_left =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{-((dim[0]+extra_space)/2),
                        ((dim[1]+extra_space)/2)});
            std::shared_ptr<Eigen::Vector2d> upper_right =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{(dim[0]+extra_space)/2,
                        ((extra_space+dim[1])/2)});
            std::shared_ptr<Eigen::Vector2d> lower_right =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{((extra_space+dim[0])/2),
                        -((extra_space+dim[1])/2)});
            std::shared_ptr<Eigen::Vector2d> lower_left =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{-((extra_space+dim[0])/2),
                        -((extra_space+dim[1])/2)});
            points.push_back(lower_left);
            points.push_back(upper_left);
            points.push_back(upper_right);
            points.push_back(lower_right);
            init_points.push_back(*lower_left);
            init_points.push_back(*upper_left);
            init_points.push_back(*upper_right);
            init_points.push_back(*lower_right);
            update_robot_position(Eigen::Vector2d{0, 0}, init_theta);
        }

        void update_robot_position(Eigen::Vector2d translation, float theta)
        {

            this->theta = theta;
            // translation and rotation are relative to the origin
            rotation_matrix = Eigen::Rotation2D<double>(theta);
            //translation the center point, we do not need to rotate because
            //in the RRT implememntation, translation is the robot's center
            //point.
            position = translation;
            // update center position
            for(int i=0; i<init_points.size(); i++)
            {
                // here, we need to rotate with respect to the robot's
                // body frame and then translate
                *(points[i]) =  rotation_matrix * *(points[i]);
                *(points[i]) = init_points[i] + translation;
                //std::cout<<*(points[i])<<std::endl;
                //std::cout<<init_points[i]<<std::endl;
            }
        }
    };

    struct Obstacle
    {
        // Assume the obstacle is rectangular

        // workspace representation
        std::vector<double> dim;
        double theta;
        Eigen::Vector2d position;
        std::vector<std::shared_ptr<Eigen::Vector2d>> points;  // points representing the obstacle in workspace

        // e.g. cspace_repr[0] gives the polygon in c-space and repr[0][0] gives the first point of that polygon
        // The order is in counter-clockwise
        std::vector<std::vector<Eigen::Vector2d>> cspace_repr{};
        // TODO: DO THE SAME THING AS THE ROBOT CASE
        Obstacle(std::vector<double> dim, double init_theta, Eigen::Vector2d init_pos):
            position(init_pos), theta(init_theta), dim(dim)
        {
            std::shared_ptr<Eigen::Vector2d> upper_left =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{init_pos(0)-(dim[0]/2), init_pos(1)+(dim[1]/2)});
            std::shared_ptr<Eigen::Vector2d> upper_right =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{init_pos(0)+(dim[0]/2), init_pos(1)+(dim[1]/2)});
            std::shared_ptr<Eigen::Vector2d> lower_right =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{init_pos(0)+(dim[0]/2), init_pos(1)-(dim[1]/2)});
            std::shared_ptr<Eigen::Vector2d> lower_left =
                std::make_shared<Eigen::Vector2d>(Eigen::Vector2d{init_pos(0)-(dim[0]/2), init_pos(1)-(dim[1]/2)});
            points.push_back(lower_left);
            points.push_back(upper_left);
            points.push_back(upper_right);
            points.push_back(lower_right);
        }

    };
}

#endif
