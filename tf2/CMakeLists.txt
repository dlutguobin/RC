set(PROJECT_NAME tf2)

# export user definitions

#CPP Libraries
add_library(tf2 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
target_link_libraries(tf2 ${Boost_LIBRARIES} console_bridge})

