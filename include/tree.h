#ifndef TREE_H
#define TREE_H
#include "node.h"
#include <vector>
#include <iostream>
using namespace std;


class Tree
{
    public:
        vector<Node*>   node_list;
        int             num_nodes;
        Node            root_node;
        // constructor
        Tree(Node &root);
        //member functions
        void add_node(Node *node, Node *parent);
        friend std::ostream& operator<<(std::ostream &os, Tree &tree);

};

#endif
