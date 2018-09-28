#include "rrt.h"
#include <fstream>
#include <boost/assign/std/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <algorithm>
#include <iterator>
#include <math.h>
#define MAX_ANGLE 2 * M_PI
#define MIN_ANGLE 0.0

RRT::RRT(double growth_param, Node &root_node, Robot &robot, int map_height,
            int map_width, vector<Obstacle*> obstacle_list, Vector3d goal)
    :tree(root_node), robot(robot), obstacle_list(obstacle_list), goal(goal)
{

    // this can be configured so that the configuration space
    // is larger than 2.
    this->map_height = map_height;
    this->map_width = map_width;
    this->growth_param = growth_param;
}

Node* RRT::find_nearest_node(Node *node){
    Node *nearest_node = this->tree.node_list[0];

    double min_dist = node->calc_distance(*nearest_node);
    if(this->tree.node_list.size() > 1)
    {
        for(auto i=begin(tree.node_list)+1, e=end(tree.node_list); i!=e; ++i)
        {
            double dist = (*i)->calc_distance(*node);

            if(dist < min_dist)
            {
                min_dist = dist;
                nearest_node = *i;
            }
        }
    }
    return nearest_node;
}

int RRT::calc_step(Node *parent_node, Node *random_node){
        /*
     * Calculate the nearest node to be connected with the nearest node
     * and append this node to the tree.
     * If return is 1, then the node okay, if return is 0 then
     * the node is okay, if return is 2, then it finds the goal.
     * */
    bool found_goal = false;
    // we first need to check the root position and the goal position
    if(tree.node_list[0]->calc_distance(goal)<growth_param)
    {
       return 2;
    }
    Vector3d parent_node_pos = parent_node->node_pos;
    Vector3d random_node_pos = random_node->node_pos;
    Node *n = new Node;
    n->node_pos = random_node->node_pos;
    if(parent_node->calc_distance(*random_node)>this->growth_param)
    {
        double len = parent_node->calc_distance(*random_node);
        double k_x = (random_node_pos(0) - parent_node_pos(0)) / len;
        double k_y = (random_node_pos(1) - parent_node_pos(1)) / len;
        double k_z = (random_node_pos(2) - parent_node_pos(2)) / len;
        // change the random node position
        n->node_pos(0) = parent_node_pos(0) + (k_x * growth_param);
        n->node_pos(1) = parent_node_pos(1) + (k_y * growth_param);
        n->node_pos(2) = parent_node_pos(2) + (k_z * growth_param);
    }

    bool collision = stopping_configure(parent_node->node_pos, n->node_pos);
    if(!collision)
    {
        tree.add_node(n, parent_node);
        if(n->calc_distance(goal)<growth_param)
        {
            tree.add_node(&goal, n);
            robot.update_robot_position(Vector2d{goal.node_pos(0), goal.node_pos(1)}, goal.node_pos(2));
            return 2;
        }
    }else{
        // convert robot back to the parent node pos
        robot.update_robot_position(Vector2d{parent_node->node_pos(0), parent_node->node_pos(1)}
                , parent_node->node_pos(2));
    }
    return collision;
}

Node RRT::random_sample(){
    Vector3d node_pos;
    Vector3d range(this->map_width, this->map_height, MAX_ANGLE);
    node_pos.setRandom();
    node_pos = node_pos.cwiseProduct(range);
    // clamp theta to min_angle - max_angle
    double angle = fabs(node_pos(2));
    node_pos(2) = angle;
    Node node(node_pos);
    return node;
}


bool RRT::stopping_configure(Vector3d parent_node_pos, Vector3d next_node_pos){
    /*
     * Check the current robot position with every
     * obstacle within the map
     * */
    // we need to copy the previous position values
    std::vector<Eigen::Vector2d> r_polygon;
    robot.update_robot_position(Vector2d{parent_node_pos(0), parent_node_pos(1)}, next_node_pos(2));

    for(auto p: robot.points)
    {
        r_polygon.push_back(*p);
    }
    robot.update_robot_position(Vector2d{next_node_pos(0), next_node_pos(1)}, next_node_pos(2));
    for(auto p: robot.points)
    {
        r_polygon.push_back(*p);
    }

    // construct a polygon along the previous position and next position using convex hull algorithm
    auto r_convex_hull = find_convex_hull(r_polygon);
    //std::cout<<"Afater convex hull"<<std::endl;
    std::reverse(r_convex_hull.begin(), r_convex_hull.end());
    bool collision = false;

    for(auto obs: obstacle_list){
        // construct polygons using boost geometry for both obstacles and
        // robot
        std::vector<Eigen::Vector2d> obs_points;
        for(auto p: obs->points)
        {
            obs_points.push_back(*p);
        }
        shape_polygon obs_polygon{};
        shape_polygon robot_polygon{};
        create_polygon(obs_polygon, obs_points);
        create_polygon(robot_polygon, r_convex_hull);
        deque<shape_polygon> out;
        collision = intersects(robot_polygon, obs_polygon);
        if(collision)
        {
            break;
        }
    }
    return collision;
}

void RRT::create_polygon(shape_polygon &polygon, std::vector<Eigen::Vector2d> &points)
{
    std::vector<point_xy> polygon_points;
    for(auto p: points)
    {
        polygon_points.push_back(point_xy(p(0), p(1)));
    }
    polygon_points.push_back(point_xy(points[0](0), points[0](1)));
    assign_points(polygon, polygon_points);
    //std::cout<<"Area " << area(polygon)<<std::endl;
}

int RRT::run_step(){
    // run the RRT algorithm and add nodes until find the goal
    Node node = random_sample();
    Node *nearest_node = find_nearest_node(&node);
    int succeeded = calc_step(nearest_node, &node);
    return succeeded;
}

std::vector<Node*> RRT::retreive_path()
{
    std::vector<Node*> path;
    auto current_node = &goal;
    path.push_back(current_node);
    auto parent = goal.parent;
    while(true)
    {
        parent = current_node->parent;
        if(!parent)
        {
           break;
        }else
        {
            path.push_back(parent);
            current_node = parent;
        }
    }
    return path;
}

int RRT::find_orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)
{
    float orientation = ((q(1) - p(1)) * (r(0) - q(0))) -
                        ((q(0) - p(0)) * (r(1) - q(1)));
    if (orientation == 0) return 0;
    else{
        return (orientation > 0) ? 1: 2;
    }
}


std::vector<Eigen::Vector2d> RRT::find_convex_hull(std::vector<Eigen::Vector2d> points)
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
