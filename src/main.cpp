#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "rrt.h"
#include <iostream>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "rrt_points");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 10);

    ros::Rate r(20);

    double growth_param = 0.5;
    Vector2d root_pos(0, 0);
    Node root_node(root_pos);
    const int map_height = 5;
    const int map_width = 5;
    MatrixXd map(map_width, map_height);
    RRT rrt(growth_param, root_node, map_height, map_width, map);

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
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


    visualization_msgs::Marker obs_1;
    visualization_msgs::Marker obs_2;
    obs_1.type = visualization_msgs::Marker::CYLINDER;
    obs_2.type = visualization_msgs::Marker::CUBE;
    obs_1.header.frame_id = obs_2.header.frame_id = "map";
    obs_1.ns = obs_2.ns = "obstacles";
    obs_1.id = 0;
    obs_2.id = 1;
    obs_1.action = obs_2.action = visualization_msgs::Marker::ADD;
    obs_1.scale.x = obs_1.scale.y = obs_1.scale.z = 1.0; //1x1x1 here means each side of the cube is 1m long
    obs_2.scale.x = obs_2.scale.y = obs_2.scale.z = 1.0; //1x1x1 here means the cylinder as diameter 1m and height 1m

    // Set the pose of the marker. since a side of the obstacle obs_1 is 1m as defined above, now we place the obs_1 center at (1, 2, 0.5). z-axis is height
    obs_1.pose.position.x = 1;
    obs_1.pose.position.y = 2;
    obs_1.pose.position.z = 0.5;
    obs_1.pose.orientation.x = 0.0;
    obs_1.pose.orientation.y = 0.0;
    obs_1.pose.orientation.z = 0.0;
    obs_1.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here

    obs_2.pose.position.x = -2;
    obs_2.pose.position.y = -1;
    obs_2.pose.position.z = 0.5;
    obs_2.pose.orientation = obs_1.pose.orientation;

    // Set the color red, green, blue. if not set, by default the value is 0
    obs_1.color.r = 0.0f;
    obs_1.color.g = 1.0f;
    obs_1.color.b = 0.0f;
    obs_1.color.a = 1.0;		//be sure to set alpha to something non-zero, otherwise it is transparent
    obs_2.color = obs_1.color;

    obs_1.lifetime = obs_2.lifetime = ros::Duration();

    geometry_msgs::Point p;
    p.x = root_pos[0];
    p.y = root_pos[1];
    p.z = 0;
    points.points.push_back(p);


    while (ros::ok())
    {
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
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
        //line_strip.points.push_back(added_point);
        line_list.points.push_back(parent_point);
        line_list.points.push_back(added_point);
        marker_pub.publish(points);
        marker_pub.publish(line_list);
        marker_pub.publish(obs_1);
        marker_pub.publish(obs_2);
        r.sleep();
    }

    return 0;
}
