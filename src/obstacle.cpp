#include "obstacle.h"
#include "iostream"
#include <cmath>
#define N_LINES 4

Obstacle::Obstacle(std::string type, double size, double height, VectorXd center):
    type(type), size(size), height(height), center(center)
{
    // For now, make cylinder a type of square shape
    double center_x = center[0];
    double center_y = center[1];
    std::cout<<"Center"<<center<<std::endl;
    upper_left[0] =  center_x - (size/2);
    upper_left[1] = center_y + (size/2);
    upper_right[0] = center_x +(size/2);
    upper_right[1] = center_y + (size/2);
    lower_right[0] = center_x + (size/2);
    lower_right[1] = center_y - (size/2);
    lower_left[0] = center_x - (size/2);
    lower_left[1] = center_y - (size/2);
    lines.push_back(std::vector<Vector2d>{upper_left, upper_right});
    lines.push_back(std::vector<Vector2d>{upper_left, lower_left});
    lines.push_back(std::vector<Vector2d>{upper_right, lower_right});
    lines.push_back(std::vector<Vector2d>{lower_left, lower_right});
}


bool Obstacle::collision_with_rect(Node &node_one, Node &node_two){
       // TODO: make C-OBS
       double x0 = node_one.node_pos[0];
       double x1 = node_two.node_pos[0];
       double y0 = node_one.node_pos[1];
       double y1 = node_two.node_pos[1];

       Vector2d point_one{x0, y0};
       Vector2d point_two{x1, y1};

       // test intersection
       bool intersected = false;
       for(int i=0; i<N_LINES; i++){
           intersected = collision_with_line(std::vector<Vector2d>{point_one, point_two}, lines[i]);
           if(intersected) break;
        }
       return intersected;
}



bool Obstacle::collision_with_line(std::vector<Vector2d> line_one, std::vector<Vector2d> line_two)
{

    auto q1 = line_two[0];
    auto q2 = line_two[1];
    auto p1 = line_one[0];
    auto p2 = line_one[1];
    float p0_x = p1[0];
    float p0_y = p1[1];
    float p1_x = p2[0];
    float p1_y = p2[1];
    float p2_x = q1[0];
    float p2_y = q1[1];
    float p3_x = q2[0];
    float p3_y = q2[1];
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        return 1;
    }

    return 0; // No collision

}
