cmake_minimum_required(VERSION 3.11)
project(manus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Determine target architecture for ManusSDK library selection
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
  set(MANUS_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(MANUS_ARCH "amd64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "Building for architecture: ${MANUS_ARCH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)  # LOCAL PATCH: 定位 .mcal 所在的 share 目录
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)

add_executable(manus_data_publisher 
  src/manus_data_publisher.cpp
  src/ClientPlatformSpecific.cpp
  src/ManusDataPublisher.cpp)
target_include_directories(manus_data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../ManusSDK/include)
target_link_directories(manus_data_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ManusSDK/lib/${MANUS_ARCH}/)

target_link_libraries(manus_data_publisher ManusSDK-${MANUS_ARCH} ncurses)

target_compile_features(manus_data_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS manus_data_publisher
  DESTINATION lib/${PROJECT_NAME})

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ManusSDK/lib/${MANUS_ARCH})

set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK-${MANUS_ARCH}.so)

install(FILES ${LIBRARY_FILE} DESTINATION lib/${PROJECT_NAME})

# LOCAL PATCH (not upstream): ship the .mcal calibration files
install(DIRECTORY calibration/
  DESTINATION share/${PROJECT_NAME}/calibration)

set_target_properties(manus_data_publisher PROPERTIES INSTALL_RPATH "$ORIGIN")

ament_target_dependencies(manus_data_publisher rclcpp ament_index_cpp std_msgs geometry_msgs manus_ros2_msgs)

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

ament_package()
