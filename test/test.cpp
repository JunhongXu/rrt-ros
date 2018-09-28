#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include "rrt.h"
#include "deque"
#include "cspace.h"
#include "entities.h"
#include "fstream"
#include <boost/assign/std/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/io/dsv/write.hpp>


void create_polygon(RRT::shape_polygon &polygon, std::vector<std::shared_ptr<Vector2d>> &points)
{
    std::vector<RRT::point_xy> polygon_points;
    for(auto p: points)
    {
        polygon_points.push_back(RRT::point_xy(float((*p)(0)), (*p)(1)));
    }
    polygon_points.push_back(RRT::point_xy((*points[0])(0), (*points[0])(1)));
    assign_points(polygon, polygon_points);
    std::cout<<"Area " << area(polygon)<<std::endl;
    std::cout << "Polygon " << boost::geometry::dsv(polygon) <<
          " has an area of " << boost::geometry::area(polygon) << std::endl;
}


void print_robot_rectangle(Robot &robot)
{
    std::cout<<"Robot Points"<<std::endl;
    for(auto &p: robot.points)
    {
        std::cout<<*p<<std::endl;
    }
}

TEST_CASE("Test RRT") {
    // setup node, robot, and obstacle
    double theta = 0.0;
    Vector2d root_node_pos(0, 0);
    Node root_node{Vector3d{root_node_pos(0), root_node_pos(1), theta}};
    Robot robot(3, std::vector<double>{2, 2}, theta, root_node_pos, 0.2);
    Obstacle obstacle(std::vector<double>{2, 2}, 0, Vector2d{1, 1});
    double growth_param = 0.5;
    int map_height = 5;
    int map_width = 5;
    std::vector<Obstacle*> obstacle_list{&obstacle};
    Vector3d goal{3, 3, 3};
    RRT rrt{growth_param, root_node, robot, map_height, map_width, obstacle_list, goal};
    SECTION("RRT RANDOM NODE")
    {
        for(int i=0; i<5; i++)
        {
            std::cout<<"Sample number "<<i<<std::endl;
            std::cout<<rrt.random_sample().node_pos<<std::endl;
        }
    }

    SECTION("RRT NEAREST NODE")
    {
        Node node_1{Vector3d{0, 0, 2}};
        Node node_2{Vector3d{0, 0, 3}};
        std::cout<<(rrt.find_nearest_node(&node_1)==&root_node)<<std::endl;
        rrt.tree.add_node(&node_1, &root_node);
        std::cout<<(rrt.find_nearest_node(&node_2)==&node_1)<<std::endl;
    }


    SECTION("RRT CALCULATE STEP")
    {
        std::cout<<"CALC"<<std::endl;
        Node random_node_1{Vector3d(0.5, 0.0, 3.14)};
        auto nearest_node = rrt.find_nearest_node(&random_node_1);
        rrt.calc_step(nearest_node, &random_node_1);
        std::cout<<"Robot position"<<std::endl;
        std::cout<<robot.position<<std::endl;
        print_robot_rectangle(robot);

/*        Node random_node_2{Vector3d{0, 0, 1.0}};*/
        //auto nearest_node_1 = rrt.find_nearest_node(&random_node_2);
        //rrt.calc_step(nearest_node_1, &random_node_2);
        //std::cout<<"Robot position"<<std::endl;
        //std::cout<<robot.position<<std::endl;
        //print_robot_rectangle(robot);

        //Node random_node_3{Vector3d{1, 0.0, 360}};
        //auto nearest_node_2 = rrt.find_nearest_node(&random_node_3);
        //rrt.calc_step(nearest_node_2, &random_node_3);
        //std::cout<<"Robot position"<<std::endl;
        //std::cout<<robot.position<<std::endl;
        //print_robot_rectangle(robot);

        //Node random_node_4{Vector3d{0, 10, 0.0}};
        //auto nearest_node_3 = rrt.find_nearest_node(&random_node_4);
        //rrt.calc_step(nearest_node_3, &random_node_4);
        //std::cout<<"Robot position"<<std::endl;
        //std::cout<<robot.position<<std::endl;
        /*print_robot_rectangle(robot);*/
    }


    SECTION("RRT CALCULATE COLLISION")
    {
        std::cout<<"Calculate Collision"<<std::endl;
        Node random_node_1{Vector3d(2, 2, 3.14)};
        Obstacle obs(std::vector<double>{3, 1}, 0, Vector2d{2, 6});
        robot.update_robot_position(Vector2d{random_node_1.node_pos(0),
                random_node_1.node_pos(1)}, random_node_1.node_pos(2));
        //std::reverse(robot.points.begin(), robot.points.end());
        RRT::shape_polygon r_p;
        RRT::shape_polygon o_p;
        create_polygon(r_p, robot.points);
        create_polygon(o_p, obs.points);
        std::cout<<"Intersects? "<<intersects(r_p, o_p)<<std::endl;
        //std::string reason;
        //std::cout<<is_valid(r_p, reason)<<std::endl;
        //std::cout<<reason<<std::endl;
    }

    SECTION("TEST BOOST")
    {
        using namespace boost::assign;
       typedef boost::geometry::model::d2::point_xy<double> point_xy;

        // Create points to represent a 5x5 closed polygon.
        std::vector<point_xy> points;
        points +=
          point_xy(0,0),
          point_xy(0,5),
          point_xy(5,5),
          point_xy(5,0),
          point_xy(0,0)
          ;

        std::vector<point_xy> points_2;
        points_2 +=
            point_xy(6, 0),
            point_xy(10, 0),
            point_xy(10, 16),
            point_xy(6, 0);
        // Create a polygon object and assign the points to it.
        boost::geometry::model::polygon<point_xy> polygon;
        boost::geometry::assign_points(polygon, points);

        boost::geometry::model::polygon<point_xy> polygon_2;
        boost::geometry::assign_points(polygon_2, points_2);


        std::cout << "Polygon " << boost::geometry::dsv(polygon) <<
          " has an area of " << boost::geometry::area(polygon) << std::endl;
        std::cout << "Polygon 2" << boost::geometry::dsv(polygon_2) <<
          " has an area of " << boost::geometry::area(polygon_2) << std::endl;

        std::deque<boost::geometry::model::polygon<point_xy>> output;
        boost::geometry::intersection(polygon, polygon_2, output);
        for(auto &p: output)
        {
            std::cout<<"INtersection: "<<boost::geometry::wkt(p)<<std::endl;
        }

    }
    //Vector2d root_pos(0, 0);
    //Node root_node(root_pos);
    //const int map_height = 5;
    //const int map_width = 5;
    //Obstacle obs1("cylinder", 1, 2, Vector2d(0.5, 0.5));
    //vector<Obstacle*> obstacle_list = {&obs1};//, &obs2, &obs3, &obs4, &obs5};
    //RRT rrt(growth_param, root_node, map_height, map_width, obstacle_list);
    //Vector2d root = rrt.tree.node_list[0]->node_pos;
    //for(int i =0; i<2;i++){
        //REQUIRE(root[i] == root_pos[i]);
    //}

    //SECTION("Random Configuration"){
        //Node point_one = rrt.random_sample();
        //Node point_two = rrt.random_sample();

        //REQUIRE(point_one.node_pos != point_two.node_pos);

        //for(int i=0; i<100; i++){
            //Node p = rrt.random_sample();
            //REQUIRE(p.node_pos[0] < 5);
            //REQUIRE(p.node_pos[1] < 5);
        //}
    //}

    //SECTION("Calculate Finding Nearest Node"){
        //// the random seed gives [3.3344, 2.48629]
        //Node node_one = Node(Vector2d(1, 1));
        //std::cout<<"Tree"<<std::endl;
        //std::cout<<rrt.tree<<std::endl;
        //Node *nearest_node = rrt.find_nearest_node(&node_one);

        //REQUIRE(nearest_node->node_pos == root_pos);
        //REQUIRE(nearest_node == &root_node);
    //}

    //SECTION("Test obstacle"){
        //Node node(Vector2d(1.01, 0.5));
        //Node parent_node(Vector2d(-1, 0));
        //std::cout<<rrt.stopping_configure(parent_node, node)<<std::endl;
    //}


    //SECTION("Test ENTITIES")
    //{
        //entities::Robot robot_1{3, std::vector<double>{2, 2}, 0, Eigen::Vector2d{1, 1}};
        //for(auto p: robot_1.points)
        //{
            //std::cout<<*p<<std::endl;
        //}

        //// change theta
        //robot_1.update_robot_position(Eigen::Vector2d{0, 0}, 90);
        //std::cout<<"Update robot position"<<std::endl;
        //std::cout<<"Center position"<<std::endl;
        //std::cout<<robot_1.position<<std::endl;
        //for(auto p: robot_1.points)
        //{
            //std::cout<<*p<<std::endl;
        //}

        //std::cout<<"Add translation"<<std::endl;
        //robot_1.update_robot_position(Eigen::Vector2d{1, 1}, 90);
        //for(auto p: robot_1.points)
        //{
            //std::cout<<*p<<std::endl;
        //}

        //std::cout<<"end.........."<<std::endl;

        //entities::Obstacle obstacle{std::vector<double>{2, 3}, 0, Eigen::Vector2d{1, 1}};
        //for(auto p: obstacle.points)
        //{
            //std::cout<<*p<<std::endl;
        //}
    /*}*/


/*    SECTION("Test COBSTACLE")*/
    //{
        //entities::Robot robot_1{3, std::vector<double>{2, 2}, 0, Eigen::Vector2d{2, 2}};
        //entities::Obstacle obstacle{std::vector<double>{4, 1}, 0, Eigen::Vector2d{4, 1}};
        //std::vector<entities::Obstacle*> v{&obstacle};
        //CSpace space{&robot_1, &v, 1};
       //space.build_cspace();
       //std::cout<<"FINISHED"<<std::endl;
       //std::ofstream file("/home/junhong/robotics_ws/src/rrt_ros/scripts/test.txt");
       //if(file.is_open())
       //{
           //for(auto o: *space.obstacles)
           //{
               //int i=0;
                //for(auto plane: o->cspace_repr)
                //{
                    //file<<"Plane " << plane.size()<<std::endl;
                    //std::cout<<"PLANE"<<std::endl;
                    //for(auto p: plane)
                    //{
                        //file << p(0) << " "<<p(1)<<'\n';
                    //}
                    //i += 1;

                //}
            //}
       //}
       //file.close();

    /*}*/

}
