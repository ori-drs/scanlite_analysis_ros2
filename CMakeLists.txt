cmake_minimum_required(VERSION 3.5)
project(scanlite_analysis_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
install(
  DIRECTORY scanlite_analysis_ros2/PyCATMAUS
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY map DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scanlite_analysis_ros2 DESTINATION share/${PROJECT_NAME}) 
# Install Python scripts
install(PROGRAMS
  scanlite_analysis_ros2/segmentation_node.py
  scanlite_analysis_ros2/LiveDemoTool.py
  scanlite_analysis_ros2/DataAcqTool.py
  scanlite_analysis_ros2/rosbagtest.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
