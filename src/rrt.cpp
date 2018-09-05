#include "rrt.h"
#include <iterator>
#include <math.h>


RRT::RRT(double growth_param, Node &root_node, int map_height, int map_width, MatrixXd &map)
:tree(root_node)
{

    this->map = map;
    // this can be configured so that the configuration space
    // is larger than 2.
    this->map_height = map_height;
    this->map_width = map_width;
    this->growth_param = growth_param;
}

Node* RRT::find_nearest_node(Node *node){
    // returning a pointer to a node that
    // is the nearest node to a random configuration
    // in the tree
    Node *nearest_node = this->tree.node_list[0];
    double min_dist = node->calc_distance(*nearest_node);
    if(this->tree.node_list.size() > 1)
    {
        for(auto i=begin(tree.node_list)+1, e=end(tree.node_list); i!=e; ++i)
        {
           Node *next_node = *i;
           double dist = next_node->calc_distance(*node);

           if(dist < min_dist)
           {
               min_dist = dist;
               nearest_node = next_node;
           }
        }

    }
    return nearest_node;
}

void RRT::calc_step(Node *nearest_node, Node *random_node){
    /*
     * Calculate the nearest node to be connected with the nearest node
     * and append this node to the tree.
     * */
    //Vector2d nearest_node_pos = nearest_node->node_pos;
    double distance = nearest_node ->calc_distance(*random_node);
    std::cout<<"Distance "<<distance<<std::endl;
    if(distance < growth_param){
       Node *new_node = new Node();
       // if don't initialize a new node, it will be a loop, idk why.
       new_node->node_pos = Vector2d(random_node->node_pos[0], random_node->node_pos[1]);
       tree.add_node(new_node, nearest_node);
    }else{
        // TODO: Sign of the x and y
        //double k =
            //(random_node->node_pos[1] - nearest_node->node_pos[1])/(random_node->node_pos[0]-nearest_node->node_pos[0]);
        //double x = sqrt(pow(growth_param, 2)/(k+1));
        //double y = sqrt(pow(growth_param, 2)/(k+1))*k;
        double y_0 = random_node->node_pos[1] - nearest_node->node_pos[1];
        double x_0 = random_node->node_pos[0] - nearest_node->node_pos[0];
        double angle = atan2(y_0, x_0);
        //std::cout<<"angle "<<angle<<std::endl;
        double x = cos(angle) * abs(growth_param) + nearest_node->node_pos[0];
        double y = sin(angle) * abs(growth_param) + nearest_node->node_pos[1];
        //std::cout<<"("<<x<<","<<y<<")"<<std::endl;
        Vector2d nearest_node_pos(x, y);
        Node *node = new Node();
        node->node_pos = nearest_node_pos;
        tree.add_node(node, nearest_node);
    }
}

Node RRT::random_sample(){
    Vector2d node_pos;
    Vector2d range(this->map_width, this->map_height);
    node_pos.setRandom();
    node_pos = (node_pos + Vector2d::Ones())/2;
    node_pos = node_pos.cwiseProduct(range);
    Node node(node_pos);
    //std::cout<<tree.node_list[0]->node_pos<<" Node" << std::endl;
    return node;
}

bool RRT::check_feasible(Node &node){
    return true; // for now
}

void RRT::run(Vector2d &destination){
    // run the RRT algorithm and add nodes until find the goal
}
