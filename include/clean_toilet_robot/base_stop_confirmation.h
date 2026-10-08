#ifndef CLEAN_TOILET_ROBOT_BASE_STOP_CONFIRMATION_H
#define CLEAN_TOILET_ROBOT_BASE_STOP_CONFIRMATION_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <cstdint>
#include <string>

namespace clean_toilet_robot
{
// 停车后并行执行最短缓冲与新鲜里程计停稳确认，不把两段等待串联相加。
class BaseStopConfirmation
{
public:
    BaseStopConfirmation(ros::NodeHandle &nh, const ros::Publisher &pub,
                         const std::string &odom_topic, double min_buffer_sec,
                         double stable_sec, double timeout_sec)
        : pub_(pub), min_buffer_sec_(min_buffer_sec), stable_sec_(stable_sec),
          timeout_sec_(timeout_sec)
    {
        sub_ = nh.subscribe(odom_topic, 1, &BaseStopConfirmation::odomCallback, this);
    }

    template <typename Active>
    bool wait(Active active, const char *phase)
    {
        const auto started = ros::WallTime::now();
        const auto stop_stamp = ros::Time::now();
        const auto deadline = started + ros::WallDuration(timeout_sec_);
        ros::WallTime still_since, previous_received;
        ros::Time still_stamp, previous_stamp;
        uint64_t seen_callback = callback_sequence_;
        unsigned int distinct_frames = 0;
        ros::WallRate rate(50);
        ROS_INFO("Base stop confirmation started: phase=%s, min_buffer=%.2f s, stable=%.2f s, linear<=0.010 m/s, angular<=0.030 rad/s, timeout=%.2f s",
                 phase, min_buffer_sec_, stable_sec_, timeout_sec_);
        while (active() && ros::WallTime::now() < deadline)
        {
            pub_.publish(geometry_msgs::Twist());
            ros::spinOnce();
            // 重复、失效、断流或重新运动的反馈不得维持或推进停稳计时。
            if (!fresh() || !slowEnough() ||
                (!previous_received.isZero() && (ros::WallTime::now()-previous_received).toSec()>0.15))
            {
                still_since = ros::WallTime();
                distinct_frames = 0;
            }
            if (callback_sequence_ != seen_callback)
            {
                seen_callback = callback_sequence_;
                if (fresh() && slowEnough() && stamp_ > stop_stamp && received_ > started)
                {
                    if (still_since.isZero() || (received_-previous_received).toSec()>0.15 ||
                        (stamp_-previous_stamp).toSec()>0.15)
                    {
                        still_since = received_;
                        still_stamp = stamp_;
                        distinct_frames = 0;
                    }
                    previous_received = received_;
                    previous_stamp = stamp_;
                    ++distinct_frames;
                    const double observed_span = (stamp_-still_stamp).toSec();
                    const double arrival_span = (received_-still_since).toSec();
                    const double elapsed = (ros::WallTime::now()-started).toSec();
                    if (distinct_frames >= 3 && observed_span >= stable_sec_ &&
                        arrival_span >= stable_sec_ && elapsed >= min_buffer_sec_)
                    {
                        ROS_INFO("Base stop confirmed: phase=%s, elapsed=%.3f s, frames=%u, observed_span=%.3f s, arrival_span=%.3f s, linear=%.4f m/s, angular=%.4f rad/s",
                                 phase, elapsed, distinct_frames, observed_span, arrival_span,
                                 linear_speed_, omega_);
                        return true;
                    }
                }
                else
                {
                    still_since = ros::WallTime();
                    distinct_frames = 0;
                }
            }
            rate.sleep();
        }
        pub_.publish(geometry_msgs::Twist());
        ROS_ERROR("Base stop confirmation failed: phase=%s, reason=%s, elapsed=%.3f s, fresh=%d, linear=%.4f m/s, angular=%.4f rad/s; task stopped before lowering",
                  phase, active() ? "odometry unavailable or base did not settle" : "interrupted",
                  (ros::WallTime::now()-started).toSec(), fresh(), linear_speed_, omega_);
        return false;
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ++callback_sequence_;
        observation_valid_ = false;
        const auto &q = msg->pose.pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(norm) || std::fabs(norm-1.0)>0.1 || msg->header.stamp.isZero() ||
            !std::isfinite(msg->twist.twist.linear.x) ||
            !std::isfinite(msg->twist.twist.linear.y) ||
            !std::isfinite(msg->twist.twist.angular.z)) return;
        const double age = (ros::Time::now()-msg->header.stamp).toSec();
        if (age < -0.1 || age > 0.5 || (!stamp_.isZero() && msg->header.stamp <= stamp_)) return;
        linear_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        omega_ = msg->twist.twist.angular.z;
        stamp_ = msg->header.stamp;
        received_ = ros::WallTime::now();
        observation_valid_ = true;
    }

    bool fresh() const
    {
        const double age = (ros::Time::now()-stamp_).toSec();
        return observation_valid_ && !received_.isZero() && age >= -0.1 && age <= 0.5 &&
               (ros::WallTime::now()-received_).toSec() <= 0.5;
    }
    bool slowEnough() const { return linear_speed_ <= 0.01 && std::fabs(omega_) <= 0.03; }

    ros::Publisher pub_;
    ros::Subscriber sub_;
    double min_buffer_sec_, stable_sec_, timeout_sec_;
    double linear_speed_ = 0.0, omega_ = 0.0;
    uint64_t callback_sequence_ = 0;
    bool observation_valid_ = false;
    ros::Time stamp_;
    ros::WallTime received_;
};
}
#endif
