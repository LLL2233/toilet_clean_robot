#include <ros/ros.h>
#include "clean_toilet_robot/action_runner.h"
#include "clean_toilet_robot/home_position_only.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cmath>
#include <cstdlib>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    MoveBaseClient ac("move_base", true);
    if (!ac.waitForServer(ros::Duration(10.0)))
    {
        ROS_ERROR("Navigation server unavailable; task not started");
        return 2;
    }
    move_base_msgs::MoveBaseGoal goal;
    tf2::Quaternion quaternion;

    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;

	//长廊过度点1\2\3
    if (!trans_point(ac, pub, 2.10, 0.350601, 0.168968)) return 2;  // 过渡点1
    if (!trans_point(ac, pub, 1.86, 0.761, 1.80)) return 2;  // 过渡点2
    if (!trans_point(ac, pub, 0.875261, 1.101102, 3.14)) return 2;  // 过渡点3


    /*
    *****************************第一个点,，马桶左侧，三号点,擦拭上盖****************************************************
    */
    double hover_x = 0.785688;      //  4号过渡点 x
    double hover_y = 1.494140;      //  4号过渡点 y
    quaternion.setRPY(0, 0, M_PI); // map 坐标系下 180 度，朝向 -X
    goal.target_pose.pose.position.x = hover_x;
    goal.target_pose.pose.position.y = hover_y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    if (!wait_for_navigation(ac, pub)) return 2;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {    
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
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
        // 保持导航目标朝向，直接执行机械臂动作。
        if (!clean_toilet_robot::runArmAction(pub, "hover"))
        {
            pub.publish(geometry_msgs::Twist());
            ROS_ERROR("Arm process failed; task stopped");
            return 3;
        }
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    /*
    *****************************第二个点，左侧向右，按钮*********************************************************************
    */
    double push_btn_x = 0.156;          // 按键点x
    double push_btn_y = 1.0;         // 按键点y
    quaternion.setRPY(0, 0, 1.57); // 朝向右侧
    goal.target_pose.pose.position.x = push_btn_x;
    goal.target_pose.pose.position.y = push_btn_y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    if (!wait_for_navigation(ac, pub)) return 2;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 抓取餐具
        if (!clean_toilet_robot::runArmAction(pub, "push_btn"))
        {
            pub.publish(geometry_msgs::Twist());
            ROS_ERROR("Arm process failed; task stopped");
            return 3;
        }
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    //3号点方便导航驶入马桶刷一侧
    if (!trans_point(ac, pub, 0.900224, 1.1052991, 1.557512)) return 2;
    /*
    *****************************第三个点，右侧向前，抓马桶刷****************************************************
    */
    double grab_x =0.646848;       // 马桶刷抓取点x
    double grab_y = 1.870705;       // 马桶刷抓取点y
    quaternion.setRPY(0, 0, 3.14); // 朝向前方
    goal.target_pose.pose.position.x = grab_x;
    goal.target_pose.pose.position.y = grab_y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    if (!wait_for_navigation(ac, pub)) return 2;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (!clean_toilet_robot::runArmAction(pub, "grab"))
        {
            pub.publish(geometry_msgs::Twist());
            ROS_ERROR("Arm process failed; task stopped");
            return 3;
        }
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    /*
    *****************************第四个点，右侧向左，刷马桶****************************************************
    */
    double flush_x = 0.784471;           // 刷洗导航点 x
    double flush_y = 1.498885;           // 刷洗导航点 y
    quaternion.setRPY(0, 0, -3.119289); // 刷洗导航点朝向
    goal.target_pose.pose.position.x = flush_x;
    goal.target_pose.pose.position.y = flush_y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("flush_MoveBase Send Goal !!!");
    if (!wait_for_navigation(ac, pub)) return 2;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Flush navigation goal reached; starting arm action");

        if (!clean_toilet_robot::runArmAction(pub, "flush"))

        {

            pub.publish(geometry_msgs::Twist());

            ROS_ERROR("Arm process failed; task stopped");

            return 3;

        }

    }
    else
    {
        ROS_WARN("flush!!! The MoveBase Goal Planning Failed for some reason");
    }

    /*
    *****************************第五个点，右侧向前，放马桶刷****************************************************
    */
    goal.target_pose.pose.position.x = grab_x; // 放置点x
    goal.target_pose.pose.position.y = grab_y; // 放置点y
    quaternion.setRPY(0, 0, -3.14);                // 朝向前方
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    if (!wait_for_navigation(ac, pub)) return 2;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (!clean_toilet_robot::runArmAction(pub, "put"))
        {
            pub.publish(geometry_msgs::Twist());
            ROS_ERROR("Arm process failed; task stopped");
            return 3;
        }
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    /*
    *****************************第六个点，左侧向右，按开关****************************************************
    */
    quaternion.setRPY(0, 0, 1.572274);              // 朝向右侧
    goal.target_pose.pose.position.x = push_btn_x; // 按开关x
    goal.target_pose.pose.position.y = push_btn_y; // 按开关y
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    if (!wait_for_navigation(ac, pub)) return 2;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The MoveBase Goal Reached Successfully!!!");
        // 抓取餐具
        if (!clean_toilet_robot::runArmAction(pub, "push_btn"))
        {
            pub.publish(geometry_msgs::Twist());
            ROS_ERROR("Arm process failed; task stopped");
            return 3;
        }
    }
    else
    {
        ROS_WARN("The MoveBase Goal Planning Failed for some reason");
    }

    // 最后直接导航到原点，目标 yaw=-pi，不再单独执行到点后的转向。
    tf2_ros::Buffer home_tf_buffer;
    tf2_ros::TransformListener home_tf_listener(home_tf_buffer);
    if (!clean_toilet_robot::navigateHomePositionOnly(
            nh, pub, ac, home_tf_buffer, wait_for_navigation)) return 2;
    ROS_INFO("Back !!!!");

    return 0;
}
