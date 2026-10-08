#ifndef CLEAN_TOILET_ROBOT_TAG_RECOVERY_H
#define CLEAN_TOILET_ROBOT_TAG_RECOVERY_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

// A single bounded forward/backward dither. Call update() from the 10 Hz
// alignment loop; callbacks must continue running while the scan is active.
class TagRecovery
{
public:
    enum class Result { Ready, Waiting, Failed };

    TagRecovery(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                const ros::Publisher &velocity_pub)
        : velocity_pub_(velocity_pub)
    {
        private_nh.param("recovery_enabled", enabled_, true);
        private_nh.param("recovery_distance_m", distance_m_, 0.015);
        private_nh.param("recovery_linear_speed", linear_speed_, 0.04);
        private_nh.param("recovery_min_clearance_m", min_clearance_m_, 0.25);
        private_nh.param("recovery_stationary_sec", stationary_seconds_, 3.0);
        // Hard safety caps also apply to values supplied through ROS parameters.
        distance_m_ = std::max(0.0, std::min(0.02, distance_m_));
        linear_speed_ = std::max(0.0, std::min(0.10, linear_speed_));
        min_clearance_m_ = std::max(0.25, std::min(1.0, min_clearance_m_));
        stationary_seconds_ = std::max(2.0, std::min(10.0, stationary_seconds_));
        std::string image_topic = "/camera/color/image_raw";
        std::string odom_topic = "/odom";
        std::string scan_topic = "/scan_filtered";
        private_nh.param("recovery_image_topic", image_topic, image_topic);
        private_nh.param("recovery_odom_topic", odom_topic, odom_topic);
        private_nh.param("recovery_scan_topic", scan_topic, scan_topic);
        image_sub_ = nh.subscribe(image_topic, 1,
                                  &TagRecovery::imageCallback, this);
        odom_sub_ = nh.subscribe(odom_topic, 1, &TagRecovery::odomCallback, this);
        scan_sub_ = nh.subscribe(scan_topic, 1, &TagRecovery::scanCallback, this);
    }

    Result update(bool tag_fresh, unsigned int valid_tag_frames,
                  bool tag_aligned, bool recover_only_on_tag_loss = false)
    {
        const ros::WallTime now = ros::WallTime::now();
        if (failed_) return Result::Failed;
        const bool stable_tag = tag_fresh && valid_tag_frames >= 3;

        // A stationary base is expected once the alignment goal is reached.
        if (stable_tag && tag_aligned && !active_ && !recover_only_on_tag_loss)
        {
            stop();
            active_ = false;
            return Result::Ready;
        }

        // This watchdog is only active inside the caller's alignment loop,
        // and only after this action has actually received its target tag.
        if (tag_fresh && !seen_tag_)
        {
            if (!odomFresh(now))
            {
                stop();
                ROS_WARN_THROTTLE(1.0, "Target tag seen; waiting for fresh odometry");
                return Result::Waiting;
            }
            seen_tag_ = true;
            resetMotionAnchor(now);
        }

        if (!seen_tag_)
        {
            stop();
            return Result::Waiting;
        }

        if (!odomFresh(now))
            return fail("Odometry stopped updating during AprilTag alignment");

        // Grab keeps correcting a visible target even if small commands do not
        // move the base. Other callers retain the original stationary watchdog.
        if (recover_only_on_tag_loss && stable_tag)
        {
            if (active_)
            {
                stop();
                active_ = false;
                ROS_INFO("AprilTag stable again; stopping dither and resuming alignment");
            }
            resetMotionAnchor(now);
            return Result::Ready;
        }

        if (!active_)
        {
            const double travel = std::hypot(odom_x_ - anchor_x_, odom_y_ - anchor_y_);
            const double yaw_change = std::fabs(std::atan2(
                std::sin(yaw_ - anchor_yaw_), std::cos(yaw_ - anchor_yaw_)));
            if (travel >= 0.01 || yaw_change >= 0.03)
                resetMotionAnchor(now);

            if ((now - last_motion_).toSec() < stationary_seconds_)
            {
                if (stable_tag) return Result::Ready;
                stop();
                return Result::Waiting;
            }

            if (!enabled_ || distance_m_ <= 0.0 || linear_speed_ <= 0.0)
            {
                if (stable_tag) return Result::Ready;
                stop();
                return Result::Waiting;
            }

            if (recovery_attempts_ >= 3)
                return fail("Base still stationary after the only permitted linear dither");

            if (!imageFresh(now) || !scanFresh(now))
                return fail("Camera frames or laser scan unavailable; linear dither disabled");
            if (!clearanceSafe())
                return fail("Insufficient front or rear clearance for linear dither");

            active_ = true;
            ++recovery_attempts_;
            origin_x_ = odom_x_;
            origin_y_ = odom_y_;
            origin_yaw_ = yaw_;
            waypoint_ = 0;
            settling_ = false;
            started_ = now;
            phase_started_ = now;
            ROS_WARN("Target tag seen but base stationary for %.1f s; starting %.0f mm forward/backward dither (front %.2f m, rear %.2f m)",
                     stationary_seconds_, distance_m_ * 1000.0,
                     front_clearance_, rear_clearance_);
        }

        if (!imageFresh(now) || !odomFresh(now) || !scanFresh(now))
            return fail("Camera, odometry or laser scan stopped updating during linear dither");
        if (!clearanceSafe())
            return fail("Front or rear clearance became unsafe during linear dither");

        if ((now - started_).toSec() >= 10.0)
            return fail("AprilTag linear dither exceeded 10 s");

        // Keep the robot within a small envelope around its starting pose.
        const double dx = odom_x_ - origin_x_;
        const double dy = odom_y_ - origin_y_;
        const double forward = dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
        const double lateral = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
        const double yaw_change = std::fabs(std::atan2(
            std::sin(yaw_ - origin_yaw_), std::cos(yaw_ - origin_yaw_)));
        if (std::fabs(forward) > distance_m_ + 0.012 ||
            std::fabs(lateral) > 0.015 || yaw_change > 0.08)
            return fail("Linear dither left its allowed position or heading envelope");

        if (settling_)
        {
            stop();
            if ((now - phase_started_).toSec() >= 0.3 &&
                image_frames_ - frames_at_stop_ >= 3)
            {
                ++waypoint_;
                if (waypoint_ >= 3)
                {
                    active_ = false;
                    resetMotionAnchor(now);
                    if (stable_tag)
                    {
                        ROS_INFO("AprilTag stable after forward/backward dither; resuming alignment");
                        return Result::Ready;
                    }
                    return fail("AprilTag did not stabilize after one linear dither");
                }
                settling_ = false;
                phase_started_ = now;
            }
            return Result::Waiting;
        }

        if ((now - phase_started_).toSec() >= 2.5)
            return fail("Odometry-guided linear step did not finish in 2.5 s");

        const double target = waypoint_ == 0 ? distance_m_ :
                              waypoint_ == 1 ? -distance_m_ : 0.0;
        const double error = target - forward;
        if (std::fabs(error) <= 0.004)
        {
            stop();
            settling_ = true;
            phase_started_ = now;
            frames_at_stop_ = image_frames_;
            return Result::Waiting;
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = error > 0.0 ? linear_speed_ : -linear_speed_;
        velocity_pub_.publish(cmd);  // No rotation during recovery.
        ROS_INFO_THROTTLE(1.0, "Linear dither step %d/3: offset %.3f m, target %.3f m, cmd %.2f m/s",
                          waypoint_ + 1, forward, target, cmd.linear.x);
        return Result::Waiting;
    }

private:
    void resetMotionAnchor(const ros::WallTime &now)
    {
        anchor_x_ = odom_x_;
        anchor_y_ = odom_y_;
        anchor_yaw_ = yaw_;
        last_motion_ = now;
    }

    bool stampFresh(const ros::Time &stamp) const
    {
        if (stamp.isZero()) return false;
        const double age = (ros::Time::now() - stamp).toSec();
        return age >= -0.1 && age <= 0.5;
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
    {
        if (!stampFresh(msg->header.stamp) ||
            (!last_image_stamp_.isZero() && msg->header.stamp <= last_image_stamp_))
            return;
        last_image_stamp_ = msg->header.stamp;
        last_image_wall_ = ros::WallTime::now();
        ++image_frames_;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (!stampFresh(msg->header.stamp) ||
            (!last_odom_stamp_.isZero() && msg->header.stamp <= last_odom_stamp_))
            return;
        const auto &q = msg->pose.pose.orientation;
        const double q_norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(q_norm) || q_norm < 0.5 || q_norm > 1.5) return;
        const double yaw = tf2::getYaw(msg->pose.pose.orientation);
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        if (!std::isfinite(yaw) || !std::isfinite(x) || !std::isfinite(y)) return;
        last_odom_stamp_ = msg->header.stamp;
        last_odom_wall_ = ros::WallTime::now();
        yaw_ = yaw;
        odom_x_ = x;
        odom_y_ = y;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        if (!stampFresh(msg->header.stamp) ||
            (!last_scan_stamp_.isZero() && msg->header.stamp <= last_scan_stamp_))
            return;
        double front = std::numeric_limits<double>::infinity();
        double rear = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const double range = msg->ranges[i];
            if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max)
                continue;
            const double angle = msg->angle_min + i * msg->angle_increment;
            const double wrapped = std::atan2(std::sin(angle), std::cos(angle));
            if (std::fabs(wrapped) <= 0.7853981633974483)
                front = std::min(front, range);
            if (std::fabs(std::fabs(wrapped) - 3.141592653589793) <= 0.7853981633974483)
                rear = std::min(rear, range);
        }
        last_scan_stamp_ = msg->header.stamp;
        last_scan_wall_ = ros::WallTime::now();
        front_clearance_ = front;
        rear_clearance_ = rear;
    }

    bool imageFresh(const ros::WallTime &now) const
    {
        return !last_image_wall_.isZero() &&
               (now - last_image_wall_).toSec() <= 0.5;
    }

    bool odomFresh(const ros::WallTime &now) const
    {
        return !last_odom_wall_.isZero() &&
               (now - last_odom_wall_).toSec() <= 0.5;
    }

    bool scanFresh(const ros::WallTime &now) const
    {
        return !last_scan_wall_.isZero() &&
               (now - last_scan_wall_).toSec() <= 0.5;
    }

    bool clearanceSafe() const
    {
        return std::isfinite(front_clearance_) && std::isfinite(rear_clearance_) &&
               front_clearance_ >= min_clearance_m_ &&
               rear_clearance_ >= min_clearance_m_;
    }

    void stop() const { velocity_pub_.publish(geometry_msgs::Twist()); }

    Result fail(const char *reason)
    {
        stop();
        failed_ = true;
        ROS_ERROR("AprilTag recovery aborted: %s (step %d, front %.2f m, rear %.2f m, odom %.3f/%.3f)",
                  reason, waypoint_ + 1, front_clearance_, rear_clearance_, odom_x_, odom_y_);
        return Result::Failed;
    }

    ros::Publisher velocity_pub_;
    ros::Subscriber image_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Time last_image_stamp_;
    ros::Time last_odom_stamp_;
    ros::Time last_scan_stamp_;
    ros::WallTime last_image_wall_;
    ros::WallTime last_odom_wall_;
    ros::WallTime last_scan_wall_;
    ros::WallTime last_motion_;
    ros::WallTime started_;
    ros::WallTime phase_started_;
    unsigned long image_frames_ = 0;
    unsigned long frames_at_stop_ = 0;
    double yaw_ = 0.0;
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double anchor_x_ = 0.0;
    double anchor_y_ = 0.0;
    double anchor_yaw_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_yaw_ = 0.0;
    double front_clearance_ = 0.0;
    double rear_clearance_ = 0.0;
    double distance_m_ = 0.015;
    double linear_speed_ = 0.08;
    double min_clearance_m_ = 0.25;
    double stationary_seconds_ = 3.0;
    int waypoint_ = 0;
    bool enabled_ = true;
    bool active_ = false;
    int recovery_attempts_ = 0;
    bool seen_tag_ = false;
    bool settling_ = false;
    bool failed_ = false;
};

#endif
