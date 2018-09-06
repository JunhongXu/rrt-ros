// header guards
#ifndef RRT_H
#define RRT_H
#include "tree.h"
#include "obstacle.h"

class RRT{
    public:
        Tree tree;
        double growth_param;
        int map_height;
        int map_width;
        vector<Obstacle*> obstacle_list;
        MatrixXd map;
        // construct rrt algorithm
        RRT(double growth_param, Node &root_node, int map_height,
            int map_width, MatrixXd &map);
        RRT(double growth_param, Node &root_node, int map_height,
            int map_width, vector<Obstacle*> obstacle_list);
        // run RRT algorithm: grow the tree until reach the destination
        void run(Vector2d &destination);
        Node random_sample();
        Node* find_nearest_node(Node *node);
        void calc_step(Node *nearest_node, Node *random_node);

    private:
        bool check_feasible(Node &node);
};
#endif
