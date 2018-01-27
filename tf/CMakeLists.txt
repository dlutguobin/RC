set(PROJECT_NAME tf)

add_library(${PROJECT_NAME}
    src/cache.cpp
    src/tf.cpp
    src/transform_broadcaster.cpp
    src/transform_listener.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Debug
add_executable(tf_empty_listener src/empty_listener.cpp)
target_link_libraries(tf_empty_listener ${PROJECT_NAME})

add_executable(tf_echo src/tf_echo.cpp)
target_link_libraries(tf_echo ${PROJECT_NAME})

add_executable(tf_change_notifier src/change_notifier.cpp)
target_link_libraries(tf_change_notifier ${PROJECT_NAME})

add_executable(tf_monitor src/tf_monitor.cpp)
target_link_libraries(tf_monitor ${PROJECT_NAME})

add_executable(static_transform_publisher src/static_transform_publisher.cpp)
target_link_libraries(static_transform_publisher ${PROJECT_NAME})

