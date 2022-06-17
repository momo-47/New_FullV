cmake_minimum_required(VERSION 2.8.3)

project(New_FullV)

find_package(catkin REQUIRED
   roscpp
   rospy
   sensor_msgs
   std_msgs
   image_geometry 
   image_transport
   geometry_msgs
   nav_msgs
   tf2
   tf2_ros

)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs  image_geometry image_transport
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


add_executable (imu_3_parts_listener src/imu_3_parts.cpp)
target_link_libraries (imu_3_parts_listener ${catkin_LIBRARIES})


add_executable (mpu6050_serial_to_imu_node2 src/mpu6050_serial_to_imu_node2.cpp)
target_link_libraries (mpu6050_serial_to_imu_node2 ${catkin_LIBRARIES})

add_executable (ticks_to_odom src/ticks_to_odom.cpp)
target_link_libraries (ticks_to_odom ${catkin_LIBRARIES})


#add_executable (odom_node src/odom.cpp)
#target_link_libraries (odom_node ${catkin_LIBRARIES})


#add_executable (publish_odometry_node src/publish_odometry.cpp)
#target_link_libraries (publish_odometry_node ${catkin_LIBRARIES})


#add_executable (range_to_laserscan_node src/range_to_laserscan.cpp)
#target_link_libraries (range_to_laserscan_node ${catkin_LIBRARIES})
