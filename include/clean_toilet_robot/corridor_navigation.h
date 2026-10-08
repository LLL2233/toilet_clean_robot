#ifndef CLEAN_TOILET_ROBOT_CORRIDOR_NAVIGATION_H
#define CLEAN_TOILET_ROBOT_CORRIDOR_NAVIGATION_H

#include <ros/ros.h>
#include <toilet_route_planner/RouteControl.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>
#include <string>

namespace clean_toilet_robot {

// 起始通道只发送一个最终目标；过渡点的经过判定由规划插件以 20 Hz 监测。
// 导航失败或漏过点位时立即退出，禁止继续机械臂动作。
template <typename Client, typename FinalYaw>
bool navigateCorridor(ros::NodeHandle& nh, ros::Publisher& pub, Client& ac,
    const std::vector<geometry_msgs::Point>& via, double target_x, double target_y,
    double pass_radius, FinalYaw final_yaw) {
    if (via.empty() || !std::isfinite(target_x) || !std::isfinite(target_y) ||
        !std::isfinite(pass_radius) || pass_radius < 0.05 || pass_radius > 0.25) {
        ROS_ERROR("Corridor: invalid route parameters"); return false;
    }
    for (const auto& p : via) if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ROS_ERROR("Corridor: non-finite via point"); return false;
    }
    double timeout=120.0;
    ros::param::param<double>("~navigation_timeout",timeout,120.0);
    if (!std::isfinite(timeout) || timeout<1.0 || timeout>600.0) {
        ROS_ERROR("Corridor: navigation_timeout must be within [1,600] seconds"); return false;
    }
    ros::ServiceClient service=nh.serviceClient<toilet_route_planner::RouteControl>(
        "/move_base/ToiletRoutePlanner/route_control");
    if (!service.waitForExistence(ros::Duration(2.0))) {
        ROS_ERROR("Corridor route planner unavailable; reload navigation before starting the task");
        pub.publish(geometry_msgs::Twist()); return false;
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id="map"; goal.target_pose.header.stamp=ros::Time::now();
    goal.target_pose.pose.position.x=target_x; goal.target_pose.pose.position.y=target_y;
    // 使用最后一段路线方向，避免误用起点到终点的直线方向。
    const double approach_yaw=std::atan2(target_y-via.back().y,target_x-via.back().x);
    tf2::Quaternion q; q.setRPY(0,0,approach_yaw); goal.target_pose.pose.orientation=tf2::toMsg(q);
    const std::string id="corridor_"+std::to_string(ros::WallTime::now().toNSec());
    toilet_route_planner::RouteControl configure;
    configure.request.command=toilet_route_planner::RouteControl::Request::CONFIGURE;
    configure.request.route_id=id; configure.request.target=goal.target_pose;
    configure.request.via_points=via; configure.request.pass_radius=pass_radius;
    if (!service.call(configure) || !configure.response.ok || !configure.response.active || configure.response.failed ||
        configure.response.passed!=0 || configure.response.total!=via.size()) {
        ROS_ERROR("Corridor route configuration failed: %s",configure.response.message.c_str());
        pub.publish(geometry_msgs::Twist()); return false;
    }
    auto clear_route=[&]() {
        toilet_route_planner::RouteControl request;
        request.request.command=toilet_route_planner::RouteControl::Request::CLEAR; request.request.route_id=id;
        return service.call(request) && request.response.ok && !request.response.active;
    };
    auto stop_failed=[&](const std::string& reason) {
        ac.cancelGoal(); ac.waitForResult(ros::Duration(2.0));
        for (int i=0; i<3 && ros::ok(); ++i) {
            pub.publish(geometry_msgs::Twist()); ros::WallDuration(0.1).sleep();
        }
        if (!clear_route()) ROS_ERROR("Corridor: route cleanup failed; no following task permitted");
        ROS_ERROR("Corridor navigation stopped: %s",reason.c_str());
        return false;
    };

    ac.sendGoal(goal);
    ROS_INFO("Corridor continuous navigation: %zu via points, radius=%.3f m; only final goal=(%.6f, %.6f), approach_yaw=%.3f rad",
        via.size(),pass_radius,target_x,target_y,approach_yaw);
    const ros::WallTime deadline=ros::WallTime::now()+ros::WallDuration(timeout);
    std::size_t last_passed=0;
    while (ros::ok() && ros::WallTime::now()<deadline) {
        toilet_route_planner::RouteControl status;
        status.request.command=toilet_route_planner::RouteControl::Request::STATUS; status.request.route_id=id;
        if (!service.call(status) || !status.response.ok || !status.response.active || status.response.failed ||
            status.response.total!=via.size() || status.response.passed<last_passed || status.response.passed>via.size())
            return stop_failed("Route confirmation lost or failed: "+status.response.message);
        if (status.response.passed!=last_passed) {
            last_passed=status.response.passed;
            ROS_INFO("Corridor progress: %zu/%zu points passed",last_passed,via.size());
        }
        const auto state=ac.getState();
        if (state.isDone()) {
            if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
                return stop_failed("Navigation "+state.toString()+": "+state.getText());
            if (last_passed!=via.size()) return stop_failed("Final goal reached without passing all corridor points");
            if (!clear_route()) return stop_failed("Cannot clear corridor route before the next action");
            pub.publish(geometry_msgs::Twist()); ros::WallDuration(0.10).sleep();
            // 沿用已有最短角度转向函数，只在马桶前确认最终作业朝向。
            if (!final_yaw()) return false;
            ROS_INFO("Corridor navigation complete: all via points passed; work heading ready");
            return true;
        }
        ros::spinOnce(); ros::WallDuration(0.10).sleep();
    }
    return stop_failed("Navigation timed out or task was stopped");
}

} // namespace clean_toilet_robot
#endif
