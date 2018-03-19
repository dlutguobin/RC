# Install script for directory: /home/siasun/urdf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/siasun/urdf/x64build/urdf/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/random_numbers/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/srdfdom/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/robot_model/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/exceptions/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/kinematics_base/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/robot_state/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/transforms/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/collision_detection/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/collision_detection_fcl/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/kinematic_constraints/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/robot_trajectory/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/trajectory_processing/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/planning_scene/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/class_loader/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/pluginlib/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/planning_interface/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/planning_request_adapter/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/rdf_loader/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/robot_model_loader/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/kinematics_plugin_loader/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/geometric_shapes/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/eigen_conversions/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/rostime/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/profiler/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/kdl_kinematics_plugin/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/constraint_samplers/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/ompl/cmake_install.cmake")
  include("/home/siasun/urdf/x64build/constraint_sampler_manager_loader/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/siasun/urdf/x64build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
