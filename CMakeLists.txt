cmake_minimum_required(VERSION 2.8.3)
project(RC)

add_definitions(-DTIXML_USE_STL)

include_directories(
    include
    urdf/include
    exceptions/include
    random_numbers/include
    srdfdom/include
    robot_model/include
    robot_state/include
    transforms/include
    kinematics_base/include
    kinematic_constraints/include
    collision_detection/include
    collision_detection_fcl/include
    robot_trajectory/include
    trajectory_processing/include
    planning_scene/include/
    class_loader/include
    planning_interface/include
    planning_request_adapter/include
    rdf_loader/include
    robot_model_loader/include
    kinematics_plugin_loader/include
    pluginlib/include
    geometric_shapes/include
    eigen_conversions/include
    rostime/include
    profiler/include
    kdl_kinematics_plugin/include
    constraint_samplers/include
    constraint_sampler_manager_loader/include
    resource_retriever/include
    planning_pipeline/include

    #tf/include/
    #tf2/include/
  )
link_directories(
        urdf/lib
)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_subdirectory(urdf)
add_subdirectory(random_numbers)
add_subdirectory(srdfdom)
add_subdirectory(robot_model)
add_subdirectory(exceptions)
add_subdirectory(kinematics_base)
add_subdirectory(robot_state)
add_subdirectory(transforms)
add_subdirectory(collision_detection)
add_subdirectory(collision_detection_fcl)
add_subdirectory(kinematic_constraints)
add_subdirectory(robot_trajectory)
add_subdirectory(trajectory_processing)
add_subdirectory(planning_scene)
add_subdirectory(class_loader)
add_subdirectory(pluginlib)
add_subdirectory(planning_interface)
add_subdirectory(planning_request_adapter)
add_subdirectory(rdf_loader)
add_subdirectory(robot_model_loader)
add_subdirectory(kinematics_plugin_loader)
add_subdirectory(resource_retriever)
add_subdirectory(geometric_shapes)
add_subdirectory(eigen_conversions)
add_subdirectory(rostime)
add_subdirectory(profiler)
add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(constraint_samplers)
add_subdirectory(ompl)
add_subdirectory(constraint_sampler_manager_loader)
add_subdirectory(planning_pipeline)
add_subdirectory(planning_request_adapter_plugins)


#add_subdirectory(tf)
#add_subdirectory(tf2)

add_executable(model_test kinematic_model.cpp)
target_link_libraries(model_test tinyxml  moveit_robot_model_loader moveit_robot_model moveit_robot_state
        urdfdom_model urdfdom_world moveit_kdl_kinematics_plugin kdl_parser console_bridge)

add_executable(planner_test motion_planning.cpp)
target_link_libraries(planner_test tinyxml  moveit_robot_model_loader moveit_robot_model moveit_robot_state
        urdfdom_model urdfdom_world  kdl_parser console_bridge moveit_collision_detection
        moveit_planning_scene moveit_planning_interface moveit_ompl_planner_plugin moveit_ompl_interface
        geometric_shapes)

add_executable(pipeline_test pipeline_planing.cpp)
target_link_libraries(pipeline_test tinyxml  moveit_robot_model_loader moveit_robot_model moveit_robot_state
        urdfdom_model urdfdom_world  kdl_parser console_bridge moveit_collision_detection
        moveit_planning_scene moveit_planning_interface moveit_ompl_planner_plugin moveit_ompl_interface
        geometric_shapes moveit_planning_pipeline moveit_default_planning_request_adapter_plugins
        moveit_planning_request_adapter)