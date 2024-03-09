cmake_minimum_required(VERSION 3.8)
project(welt_communication)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stingray_core_interfaces REQUIRED)
find_package(stingray_core_communication REQUIRED)
find_package(std_srvs REQUIRED)

set(welt_communication_SRCS
        src/hardware_bridge.cpp
        src/udp_driver.cpp
        src/messages/welt.cpp
        include/welt_communication/messages/welt.h
)
add_library(${PROJECT_NAME} ${welt_communication_SRCS})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_srvs stingray_core_interfaces stingray_core_communication)

include_directories(
        include
)

add_executable(hardware_bridge
        src/hardware_bridge.cpp
        include/welt_communication/messages/welt.h
        src/messages/welt.cpp
        )
ament_target_dependencies(hardware_bridge rclcpp std_srvs stingray_core_interfaces stingray_core_communication)

add_executable(udp_driver
        src/udp_driver.cpp
        include/welt_communication/messages/welt.h
        src/messages/welt.cpp
        )
ament_target_dependencies(udp_driver Boost rclcpp std_srvs stingray_core_interfaces stingray_core_communication)

## Install executable
install(TARGETS
        hardware_bridge
        udp_driver
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

## Install headers
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)

ament_export_dependencies(rosidl_default_runtime Boost)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

ament_package()