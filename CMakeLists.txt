cmake_minimum_required(VERSION 3.5)
project(brackship_controll)

# 必要な依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ydlidar_ros2_driver REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rviz2 REQUIRED)

# Pythonスクリプトの実行権限を与えるための設定
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# launchファイルを含むパッケージのインストール
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 実行ファイルの追加
add_executable(brackship_controll src/brackship_controll.cpp src/serial.cpp)

# 必要なライブラリのリンク
ament_target_dependencies(brackship_controll
   rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  ydlidar_ros2_driver
)

# launchファイルをインストールする
install(TARGETS brackship_controll
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
