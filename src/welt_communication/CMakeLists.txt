cmake_minimum_required(VERSION 3.8)
project(welt_communication)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(serial REQUIRED)
find_package(stingray_core_interfaces REQUIRED)
find_package(stingray_core_communication REQUIRED)

include_directories(include)

# add_library(${PROJECT_NAME}
#         include/welt_communication/messages/welt.h
#         src/messages/welt.cpp
# )
# target_link_libraries(${PROJECT_NAME} rclcpp Boost std_msgs std_srvs serial stingray_core_interfaces stingray_core_communication)
# install(
#         DIRECTORY include/${PROJECT_NAME}
#         DESTINATION include
# )

add_executable(hardware_bridge_node
        src/hardware_bridge_node.cpp
        include/welt_communication/messages/welt.h
        src/messages/welt.cpp
        )
ament_target_dependencies(hardware_bridge_node rclcpp std_msgs std_srvs serial stingray_core_interfaces stingray_core_communication)

add_executable(udp_driver_node
        src/udp_driver_node.cpp
        include/welt_communication/messages/welt.h
        src/messages/welt.cpp
        )
ament_target_dependencies(udp_driver_node rclcpp Boost std_msgs std_srvs serial stingray_core_interfaces stingray_core_communication)


## Install executable
install(TARGETS
        hardware_bridge_node
        udp_driver_node
        # ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        # EXPORT ${PROJECT_NAME}
        # LIBRARY DESTINATION lib
        # ARCHIVE DESTINATION lib
        # RUNTIME DESTINATION bin
        # INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies(rosidl_default_runtime Boost)
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

ament_package()