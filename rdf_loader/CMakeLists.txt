set(MOVEIT_LIB_NAME moveit_rdf_loader)

add_library(${MOVEIT_LIB_NAME} src/rdf_loader.cpp)
#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} urdf srdfdom)

#install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
#install(DIRECTORY include/ DESTINATION include)
