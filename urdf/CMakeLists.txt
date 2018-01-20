add_definitions(-DTIXML_USE_STL)
include_directories(
  include
  )
link_directories(
    lib
    )

add_library(urdfdom_world SHARED src/urdf_parser/pose.cpp src/urdf_parser/model.cpp src/urdf_parser/link.cpp src/urdf_parser/joint.cpp src/urdf_parser/world.cpp)
target_link_libraries(urdfdom_world tinyxml console_bridge)
    
add_library(urdfdom_model SHARED src/urdf_parser/pose.cpp src/urdf_parser/model.cpp src/urdf_parser/link.cpp src/urdf_parser/joint.cpp)
target_link_libraries(urdfdom_model tinyxml console_bridge)

add_library(urdf SHARED src/model.cpp)
target_link_libraries(urdf tinyxml urdfdom_model)

add_library(kdl_parser SHARED src/kdl_parser.cpp)
target_link_libraries(kdl_parser tinyxml urdf orocos-kdl)

add_executable(check_urdf src/urdf_parser/check_urdf.cpp)
target_link_libraries(check_urdf urdfdom_model urdfdom_world)

add_executable(urdf_to_graphiz src/urdf_parser/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz urdfdom_model)

#add_executable(test_urdf test/test_robot_model_parser.cpp)
#target_link_libraries(test_urdf gtest gtest_main)

add_subdirectory(test)


