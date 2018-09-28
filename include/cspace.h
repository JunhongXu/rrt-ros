#ifndef CSPACE_H
#define CSPACE_H
#include "entities.h"
#include <vector>

class CSpace
{
    public:
        entities::Robot *robot;
        std::vector<entities::Obstacle*> *obstacles;
        const double resolution;
        const double n_step;
        Eigen::Vector2d translation;
        CSpace(entities::Robot *robot, std::vector<entities::Obstacle*> *obstacles, double resolution);
        void build_cspace();

        // can be moved to private
        std::vector<Eigen::Vector2d> find_convex_hull(std::vector<Eigen::Vector2d> points);
        // minkowsi sum, return a series of points in 2D plane that are the sum
        // of robot and obstacle vertices
        std::vector<Eigen::Vector2d> vertice_sum(entities::Obstacle *obstacle);
        // rotate robot by a certain angle and inverse
        void rotate_robot(double angle);

        // given three points p, q, r, find the orientation of pq and qr
        // 0: parallel, 1: clockwise, 2:counter-clockwise
        int find_orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
};


#endif
