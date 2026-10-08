#ifndef CLEAN_TOILET_ROBOT_ACTION_RUNNER_H
#define CLEAN_TOILET_ROBOT_ACTION_RUNNER_H
#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>
#include <sys/wait.h>
namespace clean_toilet_robot {
// 全流程采用 CPP 默认值；独立 launch 只在显式给定参数时覆盖。
inline bool runArmAction(ros::Publisher &pub, const std::string &action) {
    if (action != "hover" && action != "push_btn" && action != "grab" &&
        action != "flush" && action != "put") {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Unknown arm action: %s", action.c_str());
        return false;
    }
    const std::string node = "toilet_arm_" + action + "_node";
    const std::string ns = "/" + node;
    std::vector<std::string> nodes;
    if (!ros::master::getNodes(nodes) ||
        std::find(nodes.begin(), nodes.end(), ns) != nodes.end()) {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Cannot start %s: ROS master unavailable or action node already running", action.c_str());
        return false;
    }
    ros::NodeHandle nh;
    if (nh.hasParam(ns) && !nh.deleteParam(ns)) {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Cannot clear stale private parameters for %s", node.c_str());
        return false;
    }
    ROS_INFO("Starting arm %s with current CPP defaults; stale private parameters cleared", action.c_str());
    // action 来自上述固定白名单，不接收命令文本；rosrun 退出码用于传递动作失败。
    const std::string command = "rosrun clean_toilet_robot " + node + " __name:=" + node;
    const auto started = ros::WallTime::now();
    const int status = std::system(command.c_str());
    const bool success = status != -1 && WIFEXITED(status) && WEXITSTATUS(status) == 0;
    ROS_INFO("Arm %s process finished: elapsed=%.3f s, success=%d",
             action.c_str(), (ros::WallTime::now() - started).toSec(), success);
    if (!success) {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Arm %s failed; no following navigation/action permitted", action.c_str());
    }
    return success;
}
}
#endif
