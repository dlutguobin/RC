set(MOVEIT_LIB_NAME moveit_kinematics_base)

include_directories(
  include
  )
add_library(${MOVEIT_LIB_NAME} src/kinematics_base.cpp)
#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

# This line is needed to ensure that messages are done being built before this is built
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${MOVEIT_LIB_NAME} console_bridge urdfdom ${Boost_LIBRARIES})

#install(TARGETS ${MOVEIT_LIB_NAME}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#
#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
