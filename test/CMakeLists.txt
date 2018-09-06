project(rrt_ros)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(rrt_test test.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../include/rrt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/tree.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/node.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/rrt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/obstacle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/obstacle.h
    
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/tree.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/node.cpp
    )

target_include_directories(
    rrt_test
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIR}
    )

target_link_libraries(
    rrt_test
    ${catkin_LIBRARIES}
    )
