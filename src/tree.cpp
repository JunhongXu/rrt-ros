#include "tree.h"
#include <string>

Tree::Tree(Node &root):root_node(root)
{
    num_nodes += 1;
    node_list.push_back(&root);
}

void Tree::add_node(Node *node, Node *parent){
    node_list.push_back(node);
    node->parent = parent;
    num_nodes += 1;
}


std::ostream& operator<<(std::ostream &os, Tree &tree){
    std::stringstream ss;
    for(auto const node: tree.node_list){
        Node *parent = node->parent;
        ss << node->node_pos;
        while(parent != nullptr){
            auto parent_pos = parent->node_pos;
            ss << "->" << parent_pos;
            parent = parent->parent;
        }
        ss << std::endl<<std::endl;
    }
    static const std::string tree_str = ss.str();
    return os<<ss.str();
}
