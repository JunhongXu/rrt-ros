#include "cspace.h"
#define MAX_ANGLE 360

#include <iostream>

CSpace::CSpace(entities::Robot *robot, std::vector<entities::Obstacle*> *obstacles, double resolution):
    resolution(resolution), n_step(MAX_ANGLE/resolution), robot(robot), obstacles(obstacles)
{
    // make robot's lower left coner to be at (0, 0) and orientation at (1, 1)
    this->translation = Eigen::Vector2d{-robot->init_pos(0) + (robot->dim[0]/2),
        -robot->init_pos(1) + (robot->dim[1]/2)};
    robot->update_robot_position(translation, 0);
}

void CSpace::build_cspace()
{
    /* *
     * This function builds the space for each obstacle using convex hull algorithm.
     * Reference: http://courses.csail.mit.edu/6.141/spring2013/pub/lectures/Lec12-ConfigurationSpace.pdf
     * */
    for(auto obstacle: *obstacles)
    {
        int n_slice = 0;
        for(double n=0.0; n<n_step+1; n++)
        {
            double angle = n * resolution;
            // rotate robot
            rotate_robot(angle);
            // sum
            std::vector<Eigen::Vector2d> points = vertice_sum(obstacle);
            // find convex hull
            auto hull = find_convex_hull(points);
            obstacle->cspace_repr.push_back(hull);
        }
    }
}

std::vector<Eigen::Vector2d> CSpace::vertice_sum(entities::Obstacle *obstacle)
{
    std::vector<Eigen::Vector2d> points;
    for(auto robot_point: robot->points)
    {
        for(auto obstacle_point: obstacle->points)
        {
/*            std::cout<<"Robot point "<<*robot_point<<std::endl;*/
            /*std::cout<<"Obstacle point "<<*obstacle_point<<std::endl;*/
            auto p = *robot_point + *obstacle_point;
            //std::cout<<"Pnt point "<<p<<std::endl;
            points.push_back(p);
        }
    }
    return points;
}

std::vector<Eigen::Vector2d> CSpace::find_convex_hull(std::vector<Eigen::Vector2d> points)
{
    std::vector<Eigen::Vector2d> convex_hull;
    int curr_point_index = 0;
    double curr_point_x = points[curr_point_index](0);
    // find the most left point in the set
    for(int i=1; i<points.size(); i++)
    {
       if(curr_point_x > points[i](0))
       {
            curr_point_x = points[i](0);
            curr_point_index = i;
       }
       std::cout<<points[i]<<std::endl;
    }

    int next_point_index;
    int last_point = curr_point_index;
    do
    {
        convex_hull.push_back(points[curr_point_index]);
        next_point_index = (curr_point_index + 1)%points.size();
        for(int other_point_index=0; other_point_index<points.size(); other_point_index++)
        {
            int orientation = find_orientation(points[curr_point_index],
                             points[other_point_index], points[next_point_index]);
            if(orientation == 2)
            {
                next_point_index = other_point_index;
            }
        }
        curr_point_index = next_point_index;
    }while(curr_point_index != last_point);
    return convex_hull;
}


int CSpace::find_orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)
{
    float orientation = ((q(1) - p(1)) * (r(0) - q(0))) -
                        ((q(0) - p(0)) * (r(1) - q(1)));
    if (orientation == 0) return 0;
    else{
        return (orientation > 0) ? 1: 2;
    }
}



void CSpace::rotate_robot(double angle)
{
    robot->update_robot_position(this->translation, 180+angle);
}
