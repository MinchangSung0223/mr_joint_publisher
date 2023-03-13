cmake_minimum_required(VERSION 3.8)
project(mr_joint_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

file(GLOB_RECURSE MR "include/ModernRobotics/*.cpp")
file(GLOB_RECURSE MR2 "include/ModernRobotics_origin/*.cpp")
file(GLOB_RECURSE INDY7 "include/Indy7/*.cpp")
add_executable(mr_joint_publisher src/mr_joint_publisher.cpp  ${INDY7} ${MR} ${MR2})
target_include_directories(mr_joint_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ModernRobotics>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ModernRobotics_origin>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Indy7>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/json>
  $<INSTALL_INTERFACE:include>)
  target_link_libraries(mr_joint_publisher jsoncpp_lib)
  target_compile_features(mr_joint_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mr_joint_publisher
  "robot_state_publisher"
  "sensor_msgs"  
  "jsoncpp"
)
add_custom_command(TARGET mr_joint_publisher POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/MR_info.json $<TARGET_FILE_DIR:mr_joint_publisher>)
add_custom_target(copy_MR_info ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/MR_info.json)


install(TARGETS mr_joint_publisher
  DESTINATION lib/${PROJECT_NAME})

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
