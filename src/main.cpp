#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "rrt.h"
#include <iostream>
#include <tf/transform_datatypes.h>
#include <utility>
#include <tuple>

std::pair<std::vector<visualization_msgs::Marker>, std::vector<Obstacle*>>
create_obstacle(std::vector<Vector2d> center_pos, std::vector<Vector2d> dims)
{
    std::vector<visualization_msgs::Marker> obstacle_markers;
    std::vector<Obstacle*> obstacles;
    for(int i=0; i<center_pos.size(); i++)
    {
        std::vector<double> dim{dims[i](0), dims[i](1)};
        Obstacle *obs = new Obstacle(dim, 0, center_pos[i]);
        obstacles.push_back(obs);
        visualization_msgs::Marker m;
        m.type = visualization_msgs::Marker::CUBE;
        m.header.frame_id = "map";
        m.ns = "obstacle";
        m.id = i;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = dim[0];  // for now
        m.scale.y = dim[1];
        m.scale.z = 1.0;
        m.pose.position.x = (*obs).position(0);
        m.pose.position.y = (*obs).position(1);
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
        m.lifetime = ros::Duration();
        obstacle_markers.push_back(m);
    }
    auto pairs = std::make_pair(obstacle_markers, obstacles);
    return pairs;
}


std::pair<visualization_msgs::Marker, Robot> create_robot(double theta,
        Vector2d root_node_pos, std::vector<double> dim)
{
    Robot r{4, dim, theta, root_node_pos, 0.3};
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::CUBE;
    m.header.frame_id = "map";
    m.ns = "Robot";
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = dim[0];  // for now
    m.scale.y = dim[1];
    m.scale.z = 0.5;
    m.pose.position.x = r.position(0);
    m.pose.position.y = r.position(1);
    tf::quaternionTFToMsg( tf::createQuaternionFromYaw(r.theta), m.pose.orientation);
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    return std::make_pair(m, r);
}


std::vector<visualization_msgs::Marker> create_markers(Vector3d goal, Robot &robot)
{
    visualization_msgs::Marker points, line_strip, line_list, goal_pos, path_line;


    // goal position
    geometry_msgs::Point goal_p;
    goal_p.x = goal(0);
    goal_p.y = goal(1);
    goal_p.z = 0;

    visualization_msgs::Marker goal_points;
    goal_points.header.frame_id = "map";
    goal_points.id = 3;
    goal_points.type=visualization_msgs::Marker::CUBE;
    goal_points.action = visualization_msgs::Marker::ADD;
    goal_points.color.b=1.0f;
    goal_points.color.a=1.0;
    goal_points.points.push_back(goal_p);
    goal_points.header.frame_id = "map";
    goal_points.pose.position.x = goal(0);
    goal_points.pose.position.y = goal(1);
    tf::quaternionTFToMsg( tf::createQuaternionFromYaw(goal(2)), goal_points.pose.orientation);
    goal_points.scale.x = robot.dim[0];
    goal_points.scale.y = robot.dim[1];
    goal_points.scale.z = 0.5;

    // tree points
    points.id = 0;
    path_line.header.frame_id =
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
    path_line.ns =
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    path_line.action =
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    path_line.pose.orientation.w =
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.01;
    points.scale.y = 0.01;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // lines
    line_strip.id = 1;
    line_list.id = 2;
    path_line.id =3;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = path_line.type =  visualization_msgs::Marker::LINE_LIST;
    line_strip.scale.x = 0.01;
    line_list.scale.x = 0.01;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    path_line.color.a = 1.0;
    path_line.scale.x = 0.1;
    path_line.color.b = 1;
    path_line.color.g =0.6;

    std::vector<visualization_msgs::Marker> markers{goal_points, points, line_strip, line_list, path_line};
    return markers;
}


void draw(RRT &rrt, visualization_msgs::Marker &robot_marker, visualization_msgs::Marker &points,
        visualization_msgs::Marker &line_list, std::vector<visualization_msgs::Marker> &obstacle_markers,
        ros::Publisher &marker_pub, int flag)
{

    Node *added_node;
    geometry_msgs::Point added_point;

    if(flag == 2)
    {
        // we need to draw the last second point because we add two nodes into the tree
        added_node = rrt.tree.node_list.end()[-2];
        added_point.x = added_node ->node_pos[0];
        added_point.y = added_node ->node_pos[1];
        points.points.push_back(added_point);

        Node *parent = added_node->parent;
        geometry_msgs::Point parent_point;
        parent_point.x = parent->node_pos[0];
        parent_point.y = parent->node_pos[1];

        line_list.points.push_back(parent_point);
        line_list.points.push_back(added_point);
        robot_marker.pose.position.x = rrt.robot.position(0);
        robot_marker.pose.position.y = rrt.robot.position(1);
        tf::quaternionTFToMsg( tf::createQuaternionFromYaw(rrt.robot.theta), robot_marker.pose.orientation);
        marker_pub.publish(points);
        marker_pub.publish(line_list);
    }
    added_node = rrt.tree.node_list.back();
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
    for(auto o: obstacle_markers)
    {
        marker_pub.publish(o);
    }

}


//////////// Everything starts from here!! ///////////
int main( int argc, char** argv )
{
    ros::init(argc, argv, "rrt");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 10);

    ros::Rate r(30);

    // initialize robot
    double theta = 0;
    Vector2d root_node_pos(5, 0);
    Node root_node{Vector3d{root_node_pos(0), root_node_pos(1), theta}};
    double growth_param = 0.5;
    int map_height = 5;
    int map_width = 5;
    Vector3d goal{0, 0, 3};
    auto obstacle_pairs =  create_obstacle(std::vector<Vector2d>{Vector2d{0, 2.5},
            Vector2d{-2.5, 0}, Vector2d{2.5, 0}, Vector2d{-1, -2}, Vector2d{2, -2}}
            ,std::vector<Vector2d>{Vector2d{5, 1},
            Vector2d{1, 5}, Vector2d{1, 1}, Vector2d{2, 2}, Vector2d{2, 1.5}});
    auto obstacle_list = obstacle_pairs.second;
    auto obstacle_markers = obstacle_pairs.first;
    auto robot_pairs = create_robot(theta, root_node_pos, std::vector<double>{0.5, 1});
    auto robot_marker = robot_pairs.first;
    auto robot = robot_pairs.second;
    RRT rrt{growth_param, root_node, robot, map_height, map_width, obstacle_list, goal};

    auto markers = create_markers(goal, robot);
    auto goal_points = markers[0];
    auto points = markers[1];
    auto line_strip = markers[2];
    auto line_list = markers[3];
    auto path_line = markers[4];
    rrt.robot.update_robot_position(root_node_pos, theta);
    robot_marker.pose.position.x = rrt.robot.position(0);
    robot_marker.pose.position.y = rrt.robot.position(1);
    tf::quaternionTFToMsg( tf::createQuaternionFromYaw(rrt.robot.theta), robot_marker.pose.orientation);
    while (ros::ok())
    {
        goal_points.header.stamp = points.header.stamp =
            line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        for(auto o: obstacle_markers)
        {
            o.header.stamp = ros::Time::now();
        }
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.header.stamp = ros::Time::now();
        int flag = rrt.run_step();
        marker_pub.publish(robot_marker);
        marker_pub.publish(goal_points);
        draw(rrt, robot_marker, points, line_list, obstacle_markers, marker_pub, flag);
        if(flag==2) break;
        r.sleep();
    }
    std::vector<Node*> path = rrt.retreive_path();
    std::reverse(path.begin(), path.end());
    ros::Rate rate(2);
    for(int i=1; i<path.size(); i++)
    {
        auto child = path[i];
        auto parent = child->parent;
        geometry_msgs::Point added_point;
        added_point.x = child ->node_pos[0];
        added_point.y = child ->node_pos[1];
        points.points.push_back(added_point);

        parent = child->parent;
        geometry_msgs::Point parent_point;
        parent_point.x = parent->node_pos[0];
        parent_point.y = parent->node_pos[1];

        path_line.points.push_back(parent_point);
        path_line.points.push_back(added_point);
        std::cout<<"NOde"<<child->node_pos<<std::endl;
        path_line.header.stamp = ros::Time::now();

        marker_pub.publish(path_line);
        rate.sleep();
    }

    for(int i=0; i<path.size(); i++)
    {
        auto child = path[i];
        auto parent = child->parent;
        robot.update_robot_position(Vector2d{child->node_pos(0), child->node_pos(1)}, child->node_pos(2));
        geometry_msgs::Point p;
        p.x = robot.position(0);
        p.y = robot.position(1);
        robot_marker.pose.position.x = p.x;
        robot_marker.pose.position.y = p.y;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(robot.theta), robot_marker.pose.orientation);
        marker_pub.publish(robot_marker);
        rate.sleep();
    }


    return 0;
}
