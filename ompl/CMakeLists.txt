
# C++ 11 required for OMPL
#add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread serialization)

#find_package(OMPL REQUIRED)



include_directories(ompl_interface/include
                    ${Boost_INCLUDE_DIRS}
                    )

#link_directories(${OMPL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(ompl_interface)

#install(FILES ompl_interface_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
