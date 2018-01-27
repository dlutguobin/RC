set(PROJECT_NAME rostime)

add_library(rostime 
  src/duration.cpp
  src/rate.cpp
  src/time.cpp)

target_link_libraries(rostime rt ${Boost_LIBRARIES})
