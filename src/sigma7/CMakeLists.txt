cmake_minimum_required(VERSION 3.8)
project(sigma7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(sigma_main src/sigma_main.cpp)
target_include_directories(sigma_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sigma_main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sigma_main
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_kdl"
  "tf2_geometry_msgs"
)



target_link_libraries(sigma_main
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.3
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.3
  ${libusb-1.0_LIBRARIES}
)

# Set target properties
target_include_directories(sigma_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(sigma_main PUBLIC c_std_99 cxx_std_17)

install(TARGETS sigma_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY lib DESTINATION share/${PROJECT_NAME})
install(FILES lib/libdhd.so.3 DESTINATION lib)
install(FILES lib/libdrd.so.3 DESTINATION lib)

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