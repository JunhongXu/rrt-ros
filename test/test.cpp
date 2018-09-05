#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include "rrt.h"

TEST_CASE("Test RRT") {
    double growth_param = 1.0;
    Vector2d root_pos(0, 0);
    Node root_node(root_pos);
    const int map_height = 5;
    const int map_width = 5;
    MatrixXd map(map_width, map_height);
    RRT rrt(growth_param, root_node, map_height, map_width, map);
    REQUIRE(map == rrt.map);
    Vector2d root = rrt.tree.node_list[0]->node_pos;
    for(int i =0; i<2;i++){
        REQUIRE(root[i] == root_pos[i]);
    }

    SECTION("Random Configuration"){
        Node point_one = rrt.random_sample();
        Node point_two = rrt.random_sample();

        REQUIRE(point_one.node_pos != point_two.node_pos);

        for(int i=0; i<100; i++){
            Node p = rrt.random_sample();
            REQUIRE(p.node_pos[0] < 5);
            REQUIRE(p.node_pos[1] < 5);
        }
    }

    SECTION("Calculate Finding Nearest Node"){
        // the random seed gives [3.3344, 2.48629]
        Node node_one = Node(Vector2d(1, 1));
        Node *nearest_node = rrt.find_nearest_node(&node_one);

        REQUIRE(nearest_node->node_pos == root_pos);
        REQUIRE(nearest_node == &root_node);

        // test add a node to the tree
        // test add node with respect to the growth param
        rrt.calc_step(nearest_node, &node_one);
        REQUIRE(rrt.tree.node_list[1]->parent == &root_node);

        Node node_two = Node(Vector2d(-1, 1));
        nearest_node = rrt.find_nearest_node(&node_two);
        rrt.calc_step(nearest_node, &node_two);

        Node node_three = Node(Vector2d(2, -1));
        nearest_node = rrt.find_nearest_node(&node_three);
        rrt.calc_step(nearest_node, &node_three);

        Node node_four = Node(Vector2d(-4, -1));
        nearest_node = rrt.find_nearest_node(&node_four);
        rrt.calc_step(nearest_node, &node_four);
        // sample another node [0.81984, 4.15006]
       /* Node node_two = rrt.random_sample();*/
        //nearest_node = rrt.find_nearest_node(&node_two);
        //REQUIRE(nearest_node == rrt.tree.node_list[1]);

        //rrt.tree.add_node(&node_two, nearest_node);

        //// sample another node [4.44474, 0.384973]
        //Node node_three = rrt.random_sample();
        //nearest_node = rrt.find_nearest_node(&node_three);
        //rrt.calc_step(nearest_node, &node_three);
        /*REQUIRE(rrt.tree.node_list[1] == nearest_node);*/

        std::cout<<rrt.tree<<std::endl;

    }

}
