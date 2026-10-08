#ifndef CLEAN_TOILET_ROBOT_ROLLING_CORRIDOR_NAVIGATION_H
#define CLEAN_TOILET_ROBOT_ROLLING_CORRIDOR_NAVIGATION_H

#include <ros/ros.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <string>

namespace clean_toilet_robot {

// 仍由原 GlobalPlanner 和 TEB 规划、避障；三个点用于引导绕墙。
// 接近当前引导点时提前换目标，不发停车命令，也不等待中间点的朝向到位。
// 此方案只控制目标切换时机，不能保证 move_base 换目标期间完全不减速。
template <typename Client, typename FinalYaw>
bool navigateRollingCorridor(ros::Publisher& pub, Client& ac, tf2_ros::Buffer& tf_buffer,
    const std::vector<geometry_msgs::Point>& via, double target_x, double target_y,
    double pass_radius, FinalYaw final_yaw) {
    if (via.empty() || !std::isfinite(target_x) || !std::isfinite(target_y) ||
        !std::isfinite(pass_radius) || pass_radius < 0.05 || pass_radius > 0.25) {
        ROS_ERROR("Rolling corridor: invalid route parameters"); return false;
    }
    for (const auto& p : via) if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ROS_ERROR("Rolling corridor: non-finite via point"); return false;
    }
    ros::NodeHandle private_nh("~");
    double timeout=120.0;
    private_nh.param("navigation_timeout",timeout,120.0);
    if (!std::isfinite(timeout) || timeout < 1.0 || timeout > 600.0) {
        ROS_ERROR("Rolling corridor: navigation_timeout must be within [1,600] seconds"); return false;
    }
    auto stop_failed=[&](const std::string& reason) {
        ac.cancelGoal(); ac.waitForResult(ros::Duration(2.0));
        for (int i=0; i<3 && ros::ok(); ++i) {
            pub.publish(geometry_msgs::Twist()); ros::WallDuration(0.1).sleep();
        }
        ROS_ERROR("Rolling corridor stopped: %s",reason.c_str()); return false;
    };
    auto fresh_pose=[&](geometry_msgs::Point& point) {
        try {
            const auto tf=tf_buffer.lookupTransform("map","base_link",ros::Time(0),ros::Duration(0.05));
            const double age=(ros::Time::now()-tf.header.stamp).toSec();
            const auto& t=tf.transform.translation;
            const auto& q=tf.transform.rotation;
            const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
            if (tf.header.stamp.isZero() || !std::isfinite(age) || age < -0.5 || age > 0.5 ||
                !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
                !std::isfinite(norm) || std::fabs(norm-1.0) > 0.01) return false;
            point.x=t.x; point.y=t.y; point.z=t.z; return true;
        } catch (const tf2::TransformException&) { return false; }
    };
    geometry_msgs::Point robot;
    const ros::WallTime pose_deadline=ros::WallTime::now()+ros::WallDuration(1.0);
    while (ros::ok() && !fresh_pose(robot)) {
        if (ros::WallTime::now() >= pose_deadline) return stop_failed("No fresh map-to-base pose before navigation");
        ros::WallDuration(0.05).sleep();
    }
    if (!ros::ok()) return stop_failed("Task stopped before navigation");
    std::vector<geometry_msgs::Point> targets=via;
    geometry_msgs::Point last; last.x=target_x; last.y=target_y; targets.push_back(last);
    std::size_t index=0;
    auto send_goal=[&]() {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id="map"; goal.target_pose.header.stamp=ros::Time::now();
        goal.target_pose.pose.position=targets[index];
        // 使用驶向本段目标的角度，避免在引导点要求提前转到下一段的朝向。
        const double yaw=std::atan2(targets[index].y-robot.y,targets[index].x-robot.x);
        tf2::Quaternion q; q.setRPY(0,0,yaw); goal.target_pose.pose.orientation=tf2::toMsg(q);
        ac.sendGoal(goal); // 新目标由 move_base 接管，不先取消旧目标或发送零速度。
        ROS_INFO("Rolling corridor goal %zu/%zu: (%.6f, %.6f), travel_yaw=%.3f rad",
            index+1,targets.size(),targets[index].x,targets[index].y,yaw);
    };
    send_goal();
    ros::WallTime deadline=ros::WallTime::now()+ros::WallDuration(timeout), invalid_since;
    ROS_INFO("Rolling corridor: %zu guide points, switch_radius=%.3f m; final position=(%.6f, %.6f)",
        via.size(),pass_radius,target_x,target_y);
    while (ros::ok()) {
        if (ros::WallTime::now() >= deadline) return stop_failed("Navigation timed out");
        auto state=ac.getState();
        // 先处理真实导航失败，不能把失败时刚好接近引导点当作通过。
        if (state.isDone() && state != actionlib::SimpleClientGoalState::SUCCEEDED)
            return stop_failed("Navigation "+state.toString()+": "+state.getText());
        if (!fresh_pose(robot)) {
            if (invalid_since.isZero()) invalid_since=ros::WallTime::now();
            if ((ros::WallTime::now()-invalid_since).toSec() >= 0.5)
                return stop_failed("Map-to-base pose unavailable or stale");
            ros::WallDuration(0.05).sleep(); continue;
        }
        invalid_since=ros::WallTime();
        state=ac.getState();
        if (state.isDone() && state != actionlib::SimpleClientGoalState::SUCCEEDED)
            return stop_failed("Navigation "+state.toString()+": "+state.getText());
        if (index == via.size()) {
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                pub.publish(geometry_msgs::Twist()); ros::WallDuration(0.10).sleep();
                // 马桶前仍沿用原来的到点停车和最短角度调整，后续机械臂逻辑不变。
                if (!final_yaw()) return stop_failed("Final work heading was not reached");
                ROS_INFO("Rolling corridor complete: final navigation succeeded; work heading ready");
                return true;
            }
        } else {
            const double distance=std::hypot(robot.x-targets[index].x,robot.y-targets[index].y);
            if (distance <= pass_radius || state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Rolling corridor guide %zu/%zu released: distance=%.3f m, reason=%s",
                    index+1,via.size(),distance,
                    state == actionlib::SimpleClientGoalState::SUCCEEDED ? "navigation succeeded" : "within switch radius");
                ++index; send_goal(); deadline=ros::WallTime::now()+ros::WallDuration(timeout);
            }
        }
        ros::spinOnce(); ros::WallDuration(0.05).sleep();
    }
    return stop_failed("Task was stopped");
}

} // namespace clean_toilet_robot
#endif
