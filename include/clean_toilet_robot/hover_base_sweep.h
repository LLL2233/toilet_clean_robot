#ifndef CLEAN_TOILET_ROBOT_HOVER_BASE_SWEEP_H
#define CLEAN_TOILET_ROBOT_HOVER_BASE_SWEEP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <cstdint>

namespace clean_toilet_robot
{
// 三个目标都相对扫动开始时的朝向计算，避免累计误差：
// 左端为 +幅度，右端为 -幅度（从左端右转两倍幅度），最后回中。
class HoverBaseSweep
{
public:
    HoverBaseSweep(ros::NodeHandle &nh, const ros::Publisher &pub,
                   const std::string &odom_topic, double angle_deg,
                   double speed, double settle_sec, double timeout_sec, double brake_lead_deg)
        : pub_(pub), angle_(angle_deg * M_PI / 180.0), speed_(speed),
          settle_sec_(settle_sec), timeout_sec_(timeout_sec),
          positive_lead_(brake_lead_deg * M_PI / 180.0),
          negative_lead_(brake_lead_deg * M_PI / 180.0)
    {
        sub_ = nh.subscribe(odom_topic, 1, &HoverBaseSweep::odomCallback, this);
    }

    template <typename Active>
    bool run(Active active)
    {
        if (!settle(active, "before_sweep")) return false;
        origin_ = yaw_;
        origin_set_ = true;
        ROS_INFO("Hover base sweep (approximate): left %.1f deg, right %.1f deg, return toward start; speed=%.3f rad/s",
                 angle_ * 180.0 / M_PI, angle_ * 360.0 / M_PI, speed_);
        if (!turnTo(origin_ + angle_, "left", active) ||
            !turnTo(origin_ - angle_, "right", active) ||
            !turnTo(origin_, "center", active))
        {
            stop();
            return false;
        }
        ROS_INFO("Hover base sweep complete: return heading error=%+.2f deg (odometry)",
                 difference(yaw_, origin_) * 180.0 / M_PI);
        stop();
        return true;
    }

private:
    static double difference(double a, double b)
    { return std::atan2(std::sin(a - b), std::cos(a - b)); }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ++callback_sequence_;
        observation_valid_ = false;
        const auto &q = msg->pose.pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(norm) || std::fabs(norm - 1.0) > 0.1 ||
            !std::isfinite(msg->twist.twist.angular.z) ||
            !std::isfinite(msg->twist.twist.linear.x) || !std::isfinite(msg->twist.twist.linear.y) ||
            msg->header.stamp.isZero()) return;
        const double age = (ros::Time::now() - msg->header.stamp).toSec();
        if (age < -0.1 || age > 0.5 ||
            (!stamp_.isZero() && msg->header.stamp <= stamp_)) return;
        yaw_ = tf2::getYaw(q);
        omega_ = msg->twist.twist.angular.z;
        linear_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        stamp_ = msg->header.stamp;
        received_ = ros::WallTime::now();
        observation_valid_ = true;
    }

    bool fresh() const
    {
        return observation_valid_ && !received_.isZero() && (ros::WallTime::now() - received_).toSec() <= 0.5 &&
               (ros::Time::now() - stamp_).toSec() >= -0.1 &&
               (ros::Time::now() - stamp_).toSec() <= 0.5;
    }

    void stop() const { pub_.publish(geometry_msgs::Twist()); }

    bool fail(const char *reason)
    {
        stop();
        ROS_ERROR("Hover base sweep failed: %s; task stopped", reason);
        return false;
    }

    template <typename Active>
    bool settle(Active active, const char *phase)
    {
        const auto started = ros::WallTime::now();
        const auto stop_stamp = ros::Time::now();
        const auto deadline = started + ros::WallDuration(1.5 + settle_sec_);
        ros::WallTime still_since, previous_received;
        ros::Time still_stamp, previous_stamp;
        uint64_t seen_callback = callback_sequence_;
        unsigned int distinct_frames = 0;
        ros::WallRate rate(50);
        ROS_INFO("Hover base settling started: phase=%s, linear<=0.010 m/s, angular<=0.030 rad/s, stable=%.2f s, timeout=%.2f s",
                 phase, settle_sec_, 1.5 + settle_sec_);
        while (active() && ros::WallTime::now() < deadline)
        {
            stop();
            ros::spinOnce();
            // 同一帧不能推进停稳计时；中断更新或运动重新出现则重新确认。
            if (!fresh() || linear_speed_ > 0.01 || std::fabs(omega_) > 0.03 ||
                (!previous_received.isZero() && (ros::WallTime::now() - previous_received).toSec() > 0.15))
            {
                still_since = ros::WallTime();
                distinct_frames = 0;
            }
            if (callback_sequence_ != seen_callback)
            {
                seen_callback = callback_sequence_;
                // 必须是本次停车检查开始后产生且收到的有效新反馈。
                if (fresh() && stamp_ > stop_stamp && received_ > started &&
                    linear_speed_ <= 0.01 && std::fabs(omega_) <= 0.03)
                {
                    if (still_since.isZero() ||
                        (received_ - previous_received).toSec() > 0.15 ||
                        (stamp_ - previous_stamp).toSec() > 0.15)
                    {
                        still_since = received_;
                        still_stamp = stamp_;
                        distinct_frames = 0;
                    }
                    previous_received = received_;
                    previous_stamp = stamp_;
                    ++distinct_frames;
                    const double observed_span = (stamp_ - still_stamp).toSec();
                    const double arrival_span = (received_ - still_since).toSec();
                    if (distinct_frames >= 3 && observed_span >= settle_sec_ && arrival_span >= settle_sec_)
                    {
                        ROS_INFO("Hover base settled: phase=%s, elapsed=%.3f s, frames=%u, observed_span=%.3f s, arrival_span=%.3f s, linear=%.4f m/s, angular=%.4f rad/s",
                                 phase, (ros::WallTime::now()-started).toSec(), distinct_frames,
                                 observed_span, arrival_span, linear_speed_, omega_);
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
        return fail(active() ? "odometry unavailable or base did not settle" : "interrupted");
    }

    template <typename Active>
    bool turnTo(double target, const char *phase, Active active)
    {
        // 扫动只要求大致到位，不为小角度残差再次反向修正。
        // 底盘存在最小有效转动量和明显的停车延迟。
        ros::spinOnce();
        if (!fresh()) return fail("odometry stale before turning");
        if (!active()) return fail("interrupted");
        const double start = yaw_;
        const double initial_error = difference(target, yaw_);
        if (std::fabs(initial_error) <= 0.5 * M_PI / 180.0)
        {
            stop();
            ROS_INFO("Hover sweep %s: already near target, no extra fine correction", phase);
            return settle(active, phase);
        }
        const double direction = initial_error >= 0.0 ? 1.0 : -1.0;
        double &learned_lead = direction > 0.0 ? positive_lead_ : negative_lead_;
        // 提前刹停量不超过本段转角的一半，保证仍有明显扫动。
        const double lead = std::min(std::max(0.2*M_PI/180.0, learned_lead),
                                     std::fabs(initial_error) * 0.5);
        const auto deadline = ros::WallTime::now() + ros::WallDuration(timeout_sec_);
        ROS_INFO("Hover sweep %s: target=%+.2f deg, remaining=%+.2f deg, brake_lead=%.2f deg (no fine correction)",
                 phase, difference(target, origin_) * 180.0 / M_PI,
                 initial_error * 180.0 / M_PI, lead * 180.0 / M_PI);
        ros::WallRate rate(50);
        while (active())
        {
            ros::spinOnce();
            if (!fresh()) return fail("odometry lost while turning");
            if (ros::WallTime::now() >= deadline) return fail("turn timeout or base did not move");
            // 允许停车惯性造成的额外转角；已过冲时仍允许往中间返回。
            // 只有继续向外转且超出包含停车惯性的范围时，才按异常处理。
            const double relative_yaw = difference(yaw_, origin_);
            const double extra_coast = std::max(3.0*M_PI/180.0,
                std::max(positive_lead_, negative_lead_) + 2.0*M_PI/180.0);
            if (origin_set_ && std::fabs(relative_yaw) > angle_ + extra_coast &&
                direction * relative_yaw > 0.0)
                return fail("base continued outward beyond sweep/coast range");
            const double turned = direction * difference(yaw_, start);
            if (turned < -1.0*M_PI/180.0)
                return fail("odometry moved opposite to command");
            const double error = difference(target, yaw_);
            if (direction * error <= lead && turned >= 0.2*M_PI/180.0) break;
            geometry_msgs::Twist cmd;
            cmd.angular.z = direction * speed_;
            pub_.publish(cmd);
            rate.sleep();
        }
        const double brake_yaw = yaw_;
        stop();
        if (!active() || !settle(active, phase)) return false;
        const double coast = std::max(0.0, direction * difference(yaw_, brake_yaw));
        // 本轮分别记录左右转动的停车惯性，回中时参考首次左转的记录。
        learned_lead = std::min(12.0*M_PI/180.0, coast);
        ROS_INFO("Hover sweep %s stopped: actual=%+.2f deg, residual=%+.2f deg, brake_at=%+.2f deg, coast=%.2f deg; approximate target accepted",
                 phase, difference(yaw_, origin_) * 180.0 / M_PI,
                 difference(target, yaw_) * 180.0 / M_PI,
                 difference(brake_yaw, origin_) * 180.0 / M_PI, coast * 180.0 / M_PI);
        return true;
    }

    ros::Publisher pub_;
    ros::Subscriber sub_;
    double angle_, speed_, settle_sec_, timeout_sec_;
    double positive_lead_, negative_lead_;
    double yaw_ = 0.0, omega_ = 0.0, linear_speed_ = 0.0, origin_ = 0.0;
    uint64_t callback_sequence_ = 0;
    bool observation_valid_ = false;
    bool origin_set_ = false;
    ros::Time stamp_;
    ros::WallTime received_;
};
}
#endif
