cmake_minimum_required(VERSION 3.8)
project(hiwonder_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# 실행 파일 생성
add_executable(imu_driver_node src/hiwonder_imu.cpp)
# 실행파일에서 쓰는 의존성 언급
ament_target_dependencies(imu_driver_node
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  serial_driver
  io_context
)

# add_executable을 통해 만들어진 바이너리 파일을 install/wit_ros2_imu/lib/wit_ros2_imu
# 로 보냄 ros2 run은 lib 경로를 뒤져서 실행시킬 걸 찾음
install(TARGETS
  imu_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
# src에 있는 폴더 자체를 통째로 복사해서 DESTINATION으로 보냄 """"launch & rviz 용도 """"
# 아키텍처에 독립적인(어떤 CPU 에서도 읽을 수 있는 텍스트 형태의) 설정값이나 스크립트들은 
# share 폴더에 모아두는 것이 Linux FHS (Filesystem Hierarchy Standard) 의 표준
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
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

ament_package()