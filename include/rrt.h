// header guards
#ifndef RRT_H
#define RRT_H
#include "tree.h"
#include "entities.h"
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace entities;
using namespace boost::geometry;
class RRT{
    public:
        typedef model::d2::point_xy<double> point_xy;
        typedef model::polygon<point_xy> shape_polygon;
        Tree tree;
        double growth_param;
        Node goal;
        int map_height;
        int map_width;
        vector<Obstacle*> obstacle_list;
        entities::Robot &robot;
        // construct rrt algorithm
        RRT(double growth_param, Node &root_node, entities::Robot &robot, int map_height,
            int map_width, vector<entities::Obstacle*> obstacle_list, Vector3d goal);
        // run RRT algorithm: grow the tree until reach the destination
        int run_step();
        Node random_sample();
        Node* find_nearest_node(Node *node);
        std::vector<Node*> retreive_path();
        int calc_step(Node *nearest_node, Node *random_node);
        bool stopping_configure(Vector3d parent_node_pos, Vector3d next_node_pos);
        std::vector<Eigen::Vector2d> find_convex_hull(std::vector<Eigen::Vector2d> points);
        int find_orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
        void create_polygon(shape_polygon &polygon, std::vector<Eigen::Vector2d> &points);

};
#endif
