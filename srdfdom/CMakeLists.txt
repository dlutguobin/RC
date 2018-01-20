set(PACKAGE_NAME srdfdom)

find_package(Boost REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR})
#link_directories(${catkin_LIBRARY_DIRS})

#add_compile_options(-std=c++11)

#catkin_python_setup()

#catkin_package(
#  LIBRARIES ${PROJECT_NAME}
#  INCLUDE_DIRS include ${TinyXML_INCLUDE_DIRS}
#  DEPENDS console_bridge urdfdom_headers urdfdom_py
#)

add_library(${PACKAGE_NAME} 
  src/model.cpp
  src/srdf_writer.cpp
)
target_link_libraries(${PACKAGE_NAME} tinyxml console_bridge)


#install(TARGETS ${PROJECT_NAME}
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
#
#install(PROGRAMS 
#  scripts/display_srdf
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
