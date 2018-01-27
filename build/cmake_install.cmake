# Install script for directory: /home/siasun/urdf

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/siasun/urdf/build/urdf/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/random_numbers/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/srdfdom/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/robot_model/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/exceptions/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/kinematics_base/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/robot_state/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/transforms/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/collision_detection/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/collision_detection_fcl/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/kinematic_constraints/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/robot_trajectory/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/trajectory_processing/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/planning_scene/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/class_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/pluginlib/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/planning_interface/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/planning_request_adapter/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/rdf_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/robot_model_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/kinematics_plugin_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/geometric_shapes/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/eigen_conversions/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/rostime/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/build/profiler/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/siasun/urdf/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/siasun/urdf/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
