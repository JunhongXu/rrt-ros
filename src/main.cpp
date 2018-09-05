#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "rrt.h"
#include <iostream>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "rrt_points");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 10);

    ros::Rate r(10000);

    double growth_param = 1;
    Vector2d root_pos(0, 0);
    Node root_node(root_pos);
    const int map_height = 5;
    const int map_width = 5;
    MatrixXd map(map_width, map_height);
    RRT rrt(growth_param, root_node, map_height, map_width, map);

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



        // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;
    line_list.scale.x = 0.01;



        // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

        // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;


    geometry_msgs::Point p;
    p.x = root_pos[0];
    p.y = root_pos[1];
    p.z = 0;
    points.points.push_back(p);
    while (ros::ok())
    {
        Node node = rrt.random_sample();
        Node *nearest_node = rrt.find_nearest_node(&node);
        rrt.calc_step(nearest_node, &node);
        Node *added_node = rrt.tree.node_list.back();
        std::cout<<"added "<<added_node->node_pos<<std::endl;

        if(added_node->calc_distance(*added_node->parent)>growth_param+0.0001){
            std::cout<<"WARNING!!!!"<<added_node->calc_distance(*added_node->parent)<<std::endl;
            break;
        }
        geometry_msgs::Point added_point;
        added_point.x = added_node ->node_pos[0];
        added_point.y = added_node ->node_pos[1];
        points.points.push_back(added_point);

        Node *parent = added_node->parent;
        geometry_msgs::Point parent_point;
        parent_point.x = parent->node_pos[0];
        parent_point.y = parent->node_pos[1];
        line_list.points.push_back(parent_point);
        line_list.points.push_back(added_point);
        marker_pub.publish(points);
        marker_pub.publish(line_list);
        r.sleep();
    }

    return 0;
}
