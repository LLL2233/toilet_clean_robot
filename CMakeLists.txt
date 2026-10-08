cmake_minimum_required(VERSION 3.0.2)
project(clean_toilet_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  toilet_route_planner
  move_base_msgs
  tf
  upros_message
)


catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(toilet_movebase_client src/movebase_client.cpp)
target_link_libraries(toilet_movebase_client ${catkin_LIBRARIES})

add_executable(toilet_tf_echo_node src/tf_echo.cpp)
target_link_libraries(toilet_tf_echo_node ${catkin_LIBRARIES})

add_executable(toilet_arm_hover_node src/arm_hover.cpp)
target_link_libraries(toilet_arm_hover_node ${catkin_LIBRARIES})

add_executable(toilet_arm_push_btn_node src/arm_push_btn.cpp)
target_link_libraries(toilet_arm_push_btn_node ${catkin_LIBRARIES})

add_executable(toilet_arm_grab_node src/arm_grab.cpp)
target_link_libraries(toilet_arm_grab_node ${catkin_LIBRARIES})

add_executable(toilet_arm_flush_node src/arm_flush.cpp)
target_link_libraries(toilet_arm_flush_node ${catkin_LIBRARIES})

add_executable(toilet_arm_put_node src/arm_put.cpp)
target_link_libraries(toilet_arm_put_node ${catkin_LIBRARIES})

add_executable(toilet_complete_node src/complete_flow.cpp)
target_link_libraries(toilet_complete_node ${catkin_LIBRARIES})

add_executable(toilet_complete_node_ src/complete_flow_.cpp)
target_link_libraries(toilet_complete_node_ ${catkin_LIBRARIES})

foreach(action_target toilet_movebase_client toilet_tf_echo_node toilet_arm_hover_node toilet_arm_push_btn_node toilet_arm_grab_node toilet_arm_flush_node toilet_arm_put_node toilet_complete_node)
  add_dependencies(${action_target} ${catkin_EXPORTED_TARGETS})
endforeach()

# 连续通道导航依赖规划插件生成的服务类型。
add_dependencies(toilet_complete_node_ ${catkin_EXPORTED_TARGETS})
