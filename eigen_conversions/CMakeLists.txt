set(PROJECT_NAME eigen_conversions)

add_library(${PROJECT_NAME}
  src/eigen_msg.cpp
  src/eigen_kdl.cpp
)
target_link_libraries(${PROJECT_NAME} orocos-kdl)
