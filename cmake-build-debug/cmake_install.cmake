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
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  INCLUDE("/home/siasun/urdf/cmake-build-debug/urdf/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/random_numbers/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/srdfdom/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/robot_model/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/exceptions/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/kinematics_base/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/robot_state/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/transforms/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/collision_detection/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/collision_detection_fcl/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/kinematic_constraints/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/robot_trajectory/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/trajectory_processing/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/planning_scene/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/class_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/pluginlib/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/planning_interface/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/planning_request_adapter/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/rdf_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/robot_model_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/kinematics_plugin_loader/cmake_install.cmake")
  INCLUDE("/home/siasun/urdf/cmake-build-debug/geometric_shapes/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/siasun/urdf/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/siasun/urdf/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
