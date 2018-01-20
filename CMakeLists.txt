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
    kinematics_base/include
  )

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_subdirectory(urdf)
add_subdirectory(random_numbers)
add_subdirectory(srdfdom)
add_subdirectory(robot_model)
add_subdirectory(exceptions)
add_subdirectory(kinematics_base)
