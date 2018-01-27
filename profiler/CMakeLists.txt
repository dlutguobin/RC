set(MOVEIT_LIB_NAME moveit_profiler)

add_library(${MOVEIT_LIB_NAME} src/profiler.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} console_bridge urdfdom ${Boost_LIBRARIES})

