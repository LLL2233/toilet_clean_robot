#ifndef CLEAN_TOILET_ROBOT_HOME_POSITION_ONLY_H
#define CLEAN_TOILET_ROBOT_HOME_POSITION_ONLY_H

#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <string>

namespace clean_toilet_robot {

// ============================================================
// 最后一段返程专用参数保护器
//
// 返程期间临时修改：
//   yaw_goal_tolerance = 3.2 rad
//   xy_goal_tolerance  = 0.02 m
//
// 返程结束后恢复原始 TEB 参数。
// ============================================================
class HomeNavigationTolerance
{
public:
    explicit HomeNavigationTolerance(ros::NodeHandle &nh)
        : nh_(nh),
          client_(nh.serviceClient<dynamic_reconfigure::Reconfigure>(
              "/move_base/TebLocalPlannerROS/set_parameters"))
    {
    }

    ~HomeNavigationTolerance()
    {
        if (changed_)
            restore();
    }

    bool enable()
    {
        std::string planner;

        // ----------------------------------------------------
        // 确认当前局部规划器确实为 TEB
        // ----------------------------------------------------
        if (!nh_.getParam("/move_base/base_local_planner", planner) ||
            planner != "teb_local_planner/TebLocalPlannerROS")
        {
            ROS_ERROR("Home: TEB local planner is not active");
            return false;
        }

        // ----------------------------------------------------
        // 保存原始 yaw_goal_tolerance
        // ----------------------------------------------------
        if (!nh_.getParam(
                "/move_base/TebLocalPlannerROS/yaw_goal_tolerance",
                original_yaw_tolerance_) ||
            !std::isfinite(original_yaw_tolerance_))
        {
            ROS_ERROR("Home: cannot read original yaw_goal_tolerance");
            return false;
        }

        // ----------------------------------------------------
        // 保存原始 xy_goal_tolerance
        // ----------------------------------------------------
        if (!nh_.getParam(
                "/move_base/TebLocalPlannerROS/xy_goal_tolerance",
                original_xy_tolerance_) ||
            !std::isfinite(original_xy_tolerance_) ||
            original_xy_tolerance_ <= 0.0)
        {
            ROS_ERROR("Home: cannot read original xy_goal_tolerance");
            return false;
        }

        if (!client_.waitForExistence(ros::Duration(2.0)))
        {
            ROS_ERROR("Home: TEB dynamic_reconfigure service unavailable");
            return false;
        }

        // 从这里开始，一旦出现异常，析构函数负责恢复。
        changed_ = true;

        // ----------------------------------------------------
        // 返程专用参数：
        //
        // yaw = 3.2：
        // 不要求机器人最终额外原地补转。
        //
        // xy = 0.02：
        // 要求机器人真正接近 (0.03, 0)，
        // 防止因为原来的较大 xy 容忍而停得离原点过远。
        // ----------------------------------------------------
        if (!set(3.2, 0.02))
        {
            ROS_ERROR("Home: failed to enable temporary TEB tolerances");

            // 尽量立即恢复
            restore();
            return false;
        }

        ROS_INFO(
            "Home temporary TEB tolerances enabled: "
            "yaw_goal_tolerance=3.200 rad, "
            "xy_goal_tolerance=0.020 m");

        return true;
    }

    bool restore()
    {
        if (!changed_)
            return true;

        if (!set(original_yaw_tolerance_,
                 original_xy_tolerance_))
        {
            ROS_ERROR(
                "Home: failed to restore original TEB tolerances");
            return false;
        }

        changed_ = false;

        ROS_INFO(
            "Home TEB tolerances restored: "
            "yaw=%.3f rad, xy=%.3f m",
            original_yaw_tolerance_,
            original_xy_tolerance_);

        return true;
    }

private:
    bool set(double yaw_tolerance,
             double xy_tolerance)
    {
        dynamic_reconfigure::Reconfigure srv;

        dynamic_reconfigure::DoubleParameter yaw_param;
        yaw_param.name = "yaw_goal_tolerance";
        yaw_param.value = yaw_tolerance;

        dynamic_reconfigure::DoubleParameter xy_param;
        xy_param.name = "xy_goal_tolerance";
        xy_param.value = xy_tolerance;

        srv.request.config.doubles.push_back(yaw_param);
        srv.request.config.doubles.push_back(xy_param);

        if (!client_.call(srv))
        {
            ROS_ERROR(
                "Home: TEB tolerance dynamic_reconfigure call failed");
            return false;
        }

        bool yaw_ok = false;
        bool xy_ok = false;

        for (const auto &actual :
             srv.response.config.doubles)
        {
            if (actual.name == yaw_param.name &&
                std::fabs(actual.value - yaw_tolerance) < 1e-6)
            {
                yaw_ok = true;
            }

            if (actual.name == xy_param.name &&
                std::fabs(actual.value - xy_tolerance) < 1e-6)
            {
                xy_ok = true;
            }
        }

        if (!yaw_ok || !xy_ok)
        {
            ROS_ERROR(
                "Home: requested TEB tolerances were not accepted "
                "(yaw_ok=%d, xy_ok=%d)",
                yaw_ok,
                xy_ok);

            return false;
        }

        return true;
    }

    ros::NodeHandle &nh_;
    ros::ServiceClient client_;

    double original_yaw_tolerance_ = 0.0;
    double original_xy_tolerance_ = 0.0;

    bool changed_ = false;
};


// ============================================================
// 最后一段返程
//
// 重要：
// 实际任务终点仍然是原点 (0,0)。
//
// 但由于局部代价地图在 (0,0) 处将底盘轮廓判定为碰撞，
// TEB 的规划参考目标向 +x 偏移 3 cm：
//
//       planning target = (0.03, 0.00, -pi)
//
// 同时临时把 TEB xy_goal_tolerance 设为 2 cm。
//
// 因此正常情况下最终机器人距原点不会超过约：
//
//       0.03 + 0.02 = 0.05 m
//
// 最终任务层再次检查：
//
//       distance_to_origin < 0.06 m
//
// 最终朝向不做检查。
// ============================================================
template <typename Client, typename Wait>
bool navigateHomePositionOnly(
    ros::NodeHandle &nh,
    ros::Publisher &pub,
    Client &ac,
    tf2_ros::Buffer &tf_buffer,
    Wait wait)
{
    // --------------------------------------------------------
    // 返程规划参考点
    //
    // 原来的：
    //     (0.00, 0.00)
    //
    // 修改为：
    //     (0.03, 0.00)
    //
    // yaw 保持 -pi。
    // --------------------------------------------------------
    const double home_target_x = 0.03;
    const double home_target_y = 0.00;
    const double home_target_yaw = -M_PI;

    // --------------------------------------------------------
    // 最终任务要求仍然是：
    // 距离真正原点小于 6 cm。
    // --------------------------------------------------------
    const double final_origin_tolerance = 0.06;

    ROS_INFO(
        "Home navigation reference target: "
        "x=%.3f, y=%.3f, yaw=-pi",
        home_target_x,
        home_target_y);

    ROS_INFO(
        "Home final requirement: "
        "distance to true origin < %.3f m",
        final_origin_tolerance);


    // --------------------------------------------------------
    // 1. 临时修改 TEB 的终点容忍参数
    //
    // yaw_goal_tolerance = 3.2 rad
    // xy_goal_tolerance  = 0.02 m
    // --------------------------------------------------------
    HomeNavigationTolerance tolerance(nh);

    if (!tolerance.enable())
    {
        pub.publish(geometry_msgs::Twist());
        return false;
    }


    // --------------------------------------------------------
    // 2. 构造返程目标
    // --------------------------------------------------------
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x =
        home_target_x;

    goal.target_pose.pose.position.y =
        home_target_y;

    goal.target_pose.pose.position.z =
        0.0;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;

    goal.target_pose.pose.orientation.z =
        std::sin(home_target_yaw / 2.0);

    goal.target_pose.pose.orientation.w =
        std::cos(home_target_yaw / 2.0);


    // --------------------------------------------------------
    // 3. 发送目标
    //
    // 注意：
    // 碰撞检测、TEB轨迹优化、动态地图检查等全部保留。
    // 这里只是把有碰撞问题的规划终点从 x=0 改到了 x=0.03。
    // --------------------------------------------------------
    ROS_INFO(
        "Home: sending collision-checked goal "
        "(%.3f, %.3f, -pi)",
        home_target_x,
        home_target_y);

    ac.sendGoal(goal);


    // --------------------------------------------------------
    // 4. 使用原来的导航等待逻辑
    //
    // move_base 必须真正报告 SUCCEEDED。
    // 不绕过 TEB 的失败状态。
    // --------------------------------------------------------
    const bool reached =
        wait(ac, pub);


    // --------------------------------------------------------
    // 5. 无论成功失败，都明确停车
    // --------------------------------------------------------
    pub.publish(geometry_msgs::Twist());


    // --------------------------------------------------------
    // 6. 恢复原来的 TEB 参数
    // --------------------------------------------------------
    const bool restored =
        tolerance.restore();


    if (!reached)
    {
        ROS_ERROR(
            "Home: navigation to shifted reference point failed");
        return false;
    }

    if (!restored)
    {
        ROS_ERROR(
            "Home: navigation succeeded but "
            "TEB parameters could not be restored");
        return false;
    }


    // --------------------------------------------------------
    // 7. 导航完成后，检查真正的任务目标：
    //
    //     距离 (0,0) < 0.06 m
    //
    // 不以 (0.03,0) 为最终业务判定标准。
    // 不检查最终 yaw。
    // --------------------------------------------------------
    ros::WallDuration(0.10).sleep();

    try
    {
        const auto final_pose =
            tf_buffer.lookupTransform(
                "map",
                "base_link",
                ros::Time(0),
                ros::Duration(0.5));

        const double final_x =
            final_pose.transform.translation.x;

        const double final_y =
            final_pose.transform.translation.y;

        const double distance_to_origin =
            std::hypot(final_x, final_y);


        ROS_INFO(
            "Home final pose: "
            "x=%.3f, y=%.3f, "
            "distance_to_origin=%.3f m",
            final_x,
            final_y,
            distance_to_origin);


        // ----------------------------------------------------
        // 这里必须检查真正原点，而不是检查偏移后的目标点。
        // ----------------------------------------------------
        if (distance_to_origin >=
            final_origin_tolerance)
        {
            ROS_ERROR(
                "Home final verification failed: "
                "distance to origin %.3f m "
                ">= %.3f m",
                distance_to_origin,
                final_origin_tolerance);

            pub.publish(geometry_msgs::Twist());
            return false;
        }


        ROS_INFO(
            "Home position reached: "
            "distance to true origin=%.3f m; "
            "final heading accepted",
            distance_to_origin);
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_ERROR(
            "Home: cannot confirm final position: %s",
            ex.what());

        pub.publish(geometry_msgs::Twist());
        return false;
    }


    return true;
}

} // namespace clean_toilet_robot

#endif