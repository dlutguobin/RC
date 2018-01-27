set(PROJECT_NAME class_loader)

message("-- catkin not found")
set(Poco_DIR cmake)
find_package(Poco REQUIRED COMPONENTS Foundation)
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_GLOBAL_BIN_DESTINATION bin)
set(CATKIN_GLOBAL_BIN_DESTINATION bin)
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include/class_loader)

include_directories(include ${Poco_INCLUDE_DIRS})

set(${PROJECT_NAME}_SRCS
  src/class_loader.cpp
  src/class_loader_core.cpp
  src/meta_object.cpp
  src/multi_library_class_loader.cpp
)
set(${PROJECT_NAME}_HDRS
  include/class_loader/class_loader.h
  include/class_loader/class_loader_core.h
  include/class_loader/class_loader_exceptions.h
  include/class_loader/class_loader_register_macro.h
  include/class_loader/console_bridge_compatibility.h
  include/class_loader/meta_object.h
  include/class_loader/multi_library_class_loader.h
)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_HDRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} console_bridge pthread ${Poco_LIBRARIES})
