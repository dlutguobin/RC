find_package(Boost REQUIRED date_time system thread)
include_directories(${Boost_INCLUDE_DIR})

include_directories(include)

set(PACKAGE_NAME random_numbers)
add_library(${PACKAGE_NAME}
  src/random_numbers.cpp)

target_link_libraries(${PACKAGE_NAME} ${Boost_LIBRARIES})

#install(TARGETS ${PROJECT_NAME}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
