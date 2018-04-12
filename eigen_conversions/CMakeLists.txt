set(PROJECT_NAME eigen_conversions)

add_library(${PROJECT_NAME}
  src/eigen_msg.cpp
  src/eigen_kdl.cpp
)
include_directories(/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/intel_atom_baytrail-wrs-linux/usr/include)
target_link_libraries(${PROJECT_NAME} orocos-kdl)
