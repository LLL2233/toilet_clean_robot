#include <ros/ros.h>
#include "clean_toilet_robot/action_runner.h"
#include "clean_toilet_robot/home_position_only.h"
#include "clean_toilet_robot/smooth_corridor_navigation.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string>
#include <algorithm>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

// 导航失败或超时立即取消并停车，禁止继续后续作业。
bool wait_for_navigation(MoveBaseClient &ac, ros::Publisher &pub)
{
    double timeout = 120.0;
    ros::param::param<double>("~navigation_timeout", timeout, 120.0);
    if (!std::isfinite(timeout) || timeout < 1.0 || timeout > 600.0)
    {
        ROS_ERROR("navigation_timeout must be within [1, 600] seconds");
        ac.cancelGoal();
        pub.publish(geometry_msgs::Twist());
        return false;
    }
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout);
    while (ros::ok() && ros::WallTime::now() < deadline)
    {
        if (ac.waitForResult(ros::Duration(0.1)))
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                return true;
            ROS_ERROR("Navigation failed: %s; %s", ac.getState().toString().c_str(),
                      ac.getState().getText().c_str());
            break;
        }
    }
    ac.cancelGoal();
    ac.waitForResult(ros::Duration(2.0));
    for (int i = 0; i < 3 && ros::ok(); ++i)
    {
        pub.publish(geometry_msgs::Twist());
        ros::WallDuration(0.1).sleep();
    }
    ROS_ERROR("Task stopped because navigation failed, timed out, or was cancelled");
    return false;
}

bool trans_point(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw)
{
    move_base_msgs::MoveBaseGoal goal;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    if (!wait_for_navigation(ac, pub)) return false;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Planning Succeed");
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
        return false;
    }
    return true;
}

// 将角度归一化到 [-pi, pi]，保证总是按最短方向旋转。
double normalize_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// 从 TF 中读取机器人在 map 坐标系下的位置和 yaw。
bool get_robot_pose(tf2_ros::Buffer &tf_buffer, double &x, double &y, double &yaw)
{
    try
    {
        const geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
            "map", "base_link", ros::Time(0), ros::Duration(0.5));

        x = transform.transform.translation.x;
        y = transform.transform.translation.y;
        tf2::Quaternion q(transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z,
                          transform.transform.rotation.w);
        yaw = tf2::getYaw(q);
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN("Cannot get robot pose from map to base_link: %s", ex.what());
        return false;
    }
}

// move_base 到达位置后，绕原地按最短角度旋转到指定 yaw。
// 大角度时快速旋转，接近目标后自动降速，兼顾比赛速度与最终姿态精度。
bool rotate_to_yaw(ros::Publisher &pub, tf2_ros::Buffer &tf_buffer, double target_yaw)
{
    ros::Rate rate(20);

    const double yaw_tolerance = 0.035; // 约 2 度，进入此范围即停车
    const double fast_rot_speed = 1.0; // 大角度快速旋转速度 rad/s
    const double slow_rot_speed = 0.4; // 接近目标后的精调速度 rad/s
    const double slow_angle = 0.35;     // 约 20 度，进入该范围后切换慢速
    const double timeout = 10.0;

    const ros::WallTime started = ros::WallTime::now();

    while (ros::ok())
    {
        double x = 0.0, y = 0.0, current_yaw = 0.0;
        if (!get_robot_pose(tf_buffer, x, y, current_yaw))
        {
            pub.publish(geometry_msgs::Twist()); // TF 读取失败，立即停止上一条转向命令。
            if ((ros::WallTime::now() - started).toSec() > timeout)
            {
                pub.publish(geometry_msgs::Twist());
                ROS_ERROR("Rotate-to-yaw failed: TF unavailable for %.1f s", timeout);
                return false;
            }
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        const double error = normalize_angle(target_yaw - current_yaw);

        ROS_INFO_THROTTLE(0.5,
                          "Rotate-to-yaw: current=%.3f rad, target=%.3f rad, error=%.3f rad",
                          current_yaw, target_yaw, error);

        if (std::fabs(error) <= yaw_tolerance)
        {
            pub.publish(geometry_msgs::Twist());
            ROS_INFO("Rotate-to-yaw complete: final error=%.3f rad", error);
            return true;
        }

        const double speed = (std::fabs(error) > slow_angle) ? fast_rot_speed : slow_rot_speed;

        geometry_msgs::Twist cmd;
        // error > 0：逆时针；error < 0：顺时针。normalize_angle 保证始终选择最短方向。
        cmd.angular.z = (error > 0.0) ? speed : -speed;
        pub.publish(cmd);

        if ((ros::WallTime::now() - started).toSec() > timeout)
        {
            pub.publish(geometry_msgs::Twist());
            ROS_ERROR("Rotate-to-yaw timed out after %.1f s", timeout);
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    pub.publish(geometry_msgs::Twist());
    return false;
}

// 工作点统一导航：TEB 只负责先到 x/y，最终工作 yaw 由 complete_flow 自己按最短角度完成。
bool navigate_to_work_pose(MoveBaseClient &ac,
                           ros::Publisher &pub,
                           tf2_ros::Buffer &tf_buffer,
                           double target_x,
                           double target_y,
                           double target_yaw,
                           const std::string &label)
{
    double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
    if (!get_robot_pose(tf_buffer, robot_x, robot_y, robot_yaw))
    {
        ROS_ERROR("%s: cannot get robot pose before navigation", label.c_str());
        return false;
    }

    const double dx = target_x - robot_x;
    const double dy = target_y - robot_y;
    const double distance = std::hypot(dx, dy);

    // 正常情况下让 TEB 在到点时朝向行驶方向；若已经几乎在目标位置，则保持当前朝向。
    const double travel_yaw = (distance > 0.02) ? std::atan2(dy, dx) : robot_yaw;

    ROS_INFO("%s position navigation: x=%.6f, y=%.6f, travel_yaw=%.3f rad, final_yaw=%.3f rad",
             label.c_str(), target_x, target_y, travel_yaw, target_yaw);

    if (!trans_point(ac, pub, target_x, target_y, travel_yaw))
    {
        ROS_ERROR("%s: position navigation failed", label.c_str());
        return false;
    }

    // 明确停车后再接管 /cmd_vel 做最终姿态调整。
    pub.publish(geometry_msgs::Twist());
    ros::WallDuration(0.10).sleep();

    if (!rotate_to_yaw(pub, tf_buffer, target_yaw))
    {
        ROS_ERROR("%s: final yaw alignment failed", label.c_str());
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    MoveBaseClient ac("move_base", true);
    if (!ac.waitForServer(ros::Duration(10.0)))
    {
        ROS_ERROR("Navigation server unavailable; task not started");
        return 2;
    }
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;

    // 起始通道：true 只发送马桶前的最终目标，三个过渡点引导连续路径。
    // 拐角附近减速通过，不设置中间停车目标；false 保留原逐点停车方案。
    double hover_x = 0.778470;
    double hover_y = 1.500533;
    const double hover_yaw = M_PI;
    const double corridor_pass_radius = 0.15; // 过渡点经过判定半径，单位 m。
    bool corridor_continuous = true;
    ros::NodeHandle private_nh("~");
    private_nh.param("corridor_continuous", corridor_continuous, true);

    if (corridor_continuous)
    {
        std::vector<geometry_msgs::Point> corridor_points(3);
        corridor_points[0].x = 1.969060; corridor_points[0].y = 0.085111;
        corridor_points[1].x = 2.194611; corridor_points[1].y = 0.482500;
        corridor_points[2].x = 1.316461; corridor_points[2].y = 0.944787;
        if (!clean_toilet_robot::navigateSmoothCorridor(nh, pub, ac, tf_buffer, corridor_points,
                hover_x, hover_y, corridor_pass_radius,
                [&]() { return rotate_to_yaw(pub, tf_buffer, hover_yaw); })) return 2;
    }
    else
    {
        // 备用原方案：启动时加 _corridor_continuous:=false 可恢复逐点停车导航。
        if (!trans_point(ac, pub, 1.969060, 0.085111, 0.300750)) return 2;
        if (!trans_point(ac, pub, 2.194611, 0.4825, 1.627541)) return 2;
        if (!trans_point(ac, pub, 1.316461, 0.944787, 3.056942)) return 2;
        if (!navigate_to_work_pose(ac, pub, tf_buffer, hover_x, hover_y, hover_yaw, "Hover")) return 2;
    }

    ROS_INFO("Hover navigation pose reached");
    vel_msg.linear.x = 0.1;
    int count = 0;
    while (ros::ok() && count < 3)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    vel_msg = geometry_msgs::Twist(); // 前进结束，所有速度清零
    pub.publish(vel_msg);

    if (!clean_toilet_robot::runArmAction(pub, "hover"))
    {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Arm process failed; task stopped");
        return 3;
    }

    /*
    *****************************第二个点，左侧向右，按钮*********************************************************************
    */
    double push_btn_x = 0.126749;      // 按键点 x
    double push_btn_y = 1.058708;        // 按键点 y
    const double push_btn_yaw_1 = 1.57;

    if (!navigate_to_work_pose(ac, pub, tf_buffer,
                               push_btn_x, push_btn_y, push_btn_yaw_1,
                               "Push button 1")) return 2;

    ROS_INFO("Push button 1 pose reached; starting arm action");
    if (!clean_toilet_robot::runArmAction(pub, "push_btn"))
    {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Arm process failed; task stopped");
        return 3;
    }

    //3号点方便导航驶入马桶刷一侧
    // if (!trans_point(ac, pub, 0.839453, 1.507282, 1.615652)) return 2;
    /*
    *****************************第三个点，右侧向前，抓马桶刷****************************************************
    */
    double grab_x = 0.634571;      // 马桶刷抓取点 x
    double grab_y = 1.901095;      // 马桶刷抓取点 y
    const double grab_yaw = 3.14; // 抓取时最终朝向

    if (!navigate_to_work_pose(ac, pub, tf_buffer,
                               grab_x, grab_y, grab_yaw,
                               "Grab brush")) return 2;

    ROS_INFO("Grab brush pose reached; starting arm action");
    if (!clean_toilet_robot::runArmAction(pub, "grab"))
    {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Arm process failed; task stopped");
        return 3;
    }

    /*
    *****************************第四个点，右侧向左，刷马桶****************************************************
    */
    double flush_x = 0.778470;       // 刷洗导航点 x
    double flush_y = 1.500533;       // 刷洗导航点 y
    const double flush_yaw = -3.14; // 刷洗时最终朝向

    if (!navigate_to_work_pose(ac, pub, tf_buffer,
                               flush_x, flush_y, flush_yaw,
                               "Flush")) return 2;

    ROS_INFO("Flush position and final yaw reached; starting arm action");
    if (!clean_toilet_robot::runArmAction(pub, "flush"))
    {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Arm process failed; task stopped");
        return 3;
    }

    /*
    *****************************第五个点，右侧向前，放马桶刷****************************************************
    */
    const double put_yaw = -3.14; // 放置时最终朝向

    if (!navigate_to_work_pose(ac, pub, tf_buffer,
                               grab_x, grab_y, put_yaw,
                               "Put brush")) return 2;

    ROS_INFO("Put brush pose reached; starting arm action");
    if (!clean_toilet_robot::runArmAction(pub, "put"))
    {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Arm process failed; task stopped");
        return 3;
    }

    /*
    *****************************第六个点，左侧向右，按开关****************************************************
    */
    const double push_btn_yaw_2 = 1.572274;

    if (!navigate_to_work_pose(ac, pub, tf_buffer,
                               push_btn_x, push_btn_y, push_btn_yaw_2,
                               "Push button 2")) return 2;

    ROS_INFO("Push button 2 pose reached; starting arm action");
    if (!clean_toilet_robot::runArmAction(pub, "push_btn"))
    {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Arm process failed; task stopped");
        return 3;
    }

    // 最后直接导航到原点，目标 yaw=-pi，不再调用作业点的最终转向流程。
    if (!clean_toilet_robot::navigateHomePositionOnly(
            nh, pub, ac, tf_buffer, wait_for_navigation)) return 2;

    ROS_INFO("Back !!!!");

    return 0;
}
