#ifndef CLEAN_TOILET_ROBOT_TAG_ALIGNMENT_H
#define CLEAN_TOILET_ROBOT_TAG_ALIGNMENT_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <string>

namespace clean_toilet_robot
{
// Startup-aware turning shared by all toilet tasks. Each caller supplies its
// original depth target and asymmetric near/far acceptance thresholds.
class TagAlignment
{
public:
    enum class Result { Waiting, Ready, Failed };
    // 停稳后同一组三张图像：既用于完成判定，也用于动作的误差补偿。
    struct ConfirmedObservation {
        double mean_x = 0.0, mean_z = 0.0, max_abs_x = 0.0;
        double min_z = 0.0, max_z = 0.0;
        unsigned int frames = 0;
        ros::Time first_frame_stamp, last_frame_stamp;
        ros::Time first_pose_stamp, last_pose_stamp;
        std::string coordinate_frame;
    };
    bool getConfirmedObservation(ConfirmedObservation &out, std::string *reason = nullptr) const
    {
        if (reason) reason->clear();
        const auto reject = [reason](const char *message) {
            if (reason) *reason = message;
            return false;
        };
        const auto now = ros::WallTime::now();
        const ros::Time settled_stamp = std::max(stopped_stamp_, last_motion_stamp_) +
                                        ros::Duration(settle_seconds_);
        if (!confirmed_valid_) return reject("confirmation reset by motion/target invalidation or not ready");
        if (state_ != State::Observe) return reject("alignment still moving");
        if (confirmed_.frames != 3 || confirmed_.coordinate_frame.empty())
            return reject("invalid frame count or coordinate frame");
        if (odom_received_.isZero() || (now - odom_received_).toSec() > 0.5 || !stampIsFresh(odom_stamp_))
            return reject("odometry unavailable or stale");
        if ((now - stopped_at_).toSec() < settle_seconds_ || (now - last_motion_).toSec() < settle_seconds_)
            return reject("base not settled");
        // 每帧在加入窗口时已经通过 0.5 s 新鲜度检查。
        // 使用均值时以最新图像/检测判断结果是否过期，避免把最早一帧当成最新观测。
        // 同时限制三帧跨度；运动后清空窗口的保护保持原样。
        if (!stampIsFresh(confirmed_.last_frame_stamp)) return reject("latest confirmed image stale");
        if (!confirmed_.last_pose_stamp.isZero() && !stampIsFresh(confirmed_.last_pose_stamp))
            return reject("latest confirmed pose stale");
        const double image_span = (confirmed_.last_frame_stamp - confirmed_.first_frame_stamp).toSec();
        if (image_span <= 0.0 || image_span > 0.5) return reject("three-image window exceeds 0.5 s");
        if (!confirmed_.first_pose_stamp.isZero() && !confirmed_.last_pose_stamp.isZero())
        {
            const double pose_span = (confirmed_.last_pose_stamp - confirmed_.first_pose_stamp).toSec();
            if (pose_span <= 0.0 || pose_span > 0.5) return reject("three-pose window exceeds 0.5 s");
        }
        if (confirmed_.first_frame_stamp <= settled_stamp ||
            (!confirmed_.first_pose_stamp.isZero() && confirmed_.first_pose_stamp <= settled_stamp))
            return reject("confirmation predates latest settling window");
        if (!std::isfinite(confirmed_.mean_x) || !std::isfinite(confirmed_.mean_z) ||
            confirmed_.max_abs_x > tolerance() ||
            !depthWithin(confirmed_.min_z) || !depthWithin(confirmed_.max_z))
            return reject("confirmed sample outside alignment limits");
        out = confirmed_;
        return true;
    }
    TagAlignment(ros::NodeHandle &nh, const ros::Publisher &pub, double turn_speed,
                   double linear_min, double fine_tolerance, double coarse_threshold,
                   double pulse_seconds, double settle_seconds, double startup_timeout, int max_pulses,
                   const std::string &odom_topic, double target_z, double near_tolerance, double far_tolerance,
                   double hard_tolerance = 0.02)
        : pub_(pub), turn_speed_(turn_speed), linear_min_(linear_min),
          fine_tolerance_(fine_tolerance), coarse_threshold_(coarse_threshold),
          pulse_seconds_(pulse_seconds), settle_seconds_(settle_seconds), startup_timeout_(startup_timeout),
          max_pulses_(max_pulses), target_z_(target_z), near_tolerance_(near_tolerance), far_tolerance_(far_tolerance),
          hard_tolerance_(hard_tolerance)
    {
        odom_sub_ = nh.subscribe(odom_topic, 1, &TagAlignment::odomCallback, this);
    }

    bool depthWithin(double z) const
    { return z >= target_z_ - near_tolerance_ && z <= target_z_ + far_tolerance_; }

    double tolerance() const { return fallback_ ? hard_tolerance_ : fine_tolerance_; }
    void resetMotion()
    {
        // Recovery owns cmd_vel while active. Do not override its commands here.
        state_ = State::Observe;
        clearObservations();
        stopped_at_ = ros::WallTime::now();
        stopped_stamp_ = ros::Time::now();
    }

    Result update(double x, double z, const ros::Time &frame_stamp,
                  const ros::Time &pose_stamp, const std::string &frame)
    {
        const ros::WallTime now = ros::WallTime::now();
        if (odom_received_.isZero() || (now - odom_received_).toSec() > 0.5)
            return fail("AprilTag alignment odometry unavailable or stale");
        if (!std::isfinite(x) || !std::isfinite(z) || z <= 0.0 || frame.empty() ||
            !stampIsFresh(frame_stamp) || (!pose_stamp.isZero() && !stampIsFresh(pose_stamp)))
        {
            pub_.publish(geometry_msgs::Twist());
            resetMotion();
            return Result::Waiting;
        }
        if (state_ != State::Observe && frame != motion_frame_)
        {
            pub_.publish(geometry_msgs::Twist());
            resetMotion();
            ROS_WARN("AprilTag target coordinate frame changed; stopping before reassessment");
            return Result::Waiting;
        }
        // Static friction / motor startup delay must not consume the fine pulse.
        // Both coarse and fine turns require a measured, correctly directed yaw change.
        if (state_ == State::Pulse || state_ == State::Coarse)
        {
            const double signed_turn = direction_ * std::atan2(std::sin(yaw_ - start_yaw_),
                                                              std::cos(yaw_ - start_yaw_));
            if (signed_turn < -0.01)
                return fail("AprilTag turn feedback moved opposite to commanded direction");
            if (turn_started_at_.isZero() && signed_turn >= 0.003)
            {
                turn_started_at_ = now;
                ROS_INFO("AprilTag turn started: startup=%.3f s, actual_turn=%.2f deg",
                         (now - started_).toSec(), signed_turn * 180.0 / M_PI);
            }
            if (turn_started_at_.isZero() && (now - started_).toSec() >= startup_timeout_)
            {
                ROS_ERROR("AprilTag base did not start turning: speed=%.3f rad/s, waited=%.3f s, actual_turn=%.2f deg; check base response, odometry and cmd_vel publishers",
                          turn_speed_, (now - started_).toSec(), signed_turn * 180.0 / M_PI);
                return fail("AprilTag turn startup failed; stopping instead of repeating ineffective pulses");
            }
        }
        if (state_ == State::Pulse)
        {
            const double turned = std::fabs(std::atan2(std::sin(yaw_ - start_yaw_),
                                                       std::cos(yaw_ - start_yaw_)));
            const double moving_time = turn_started_at_.isZero() ? 0.0 : (now - turn_started_at_).toSec();
            if ((!turn_started_at_.isZero() && moving_time >= pulse_seconds_) || turned >= target_turn_)
            {
                ROS_INFO("AprilTag pulse stopped: actual_turn=%.2f deg, elapsed=%.3f s, moving_time=%.3f s",
                         turned * 180.0 / M_PI, (now - started_).toSec(), moving_time);
                stopAndObserve();
            }
            else publishTurn();
            return Result::Waiting;
        }
        if (state_ == State::Coarse)
        {
            const double signed_turn = direction_ * std::atan2(std::sin(yaw_ - start_yaw_),
                                                              std::cos(yaw_ - start_yaw_));
            if (std::fabs(x) <= coarse_threshold_ * 0.75 ||
                direction_ * (-x) <= 0.0 || signed_turn >= target_turn_)
                stopAndObserve();
            else publishTurn();
            return Result::Waiting;
        }
        if (state_ == State::Linear)
        {
            const double z_error = z - target_z_;
            if (std::fabs(x) > tolerance() || depthWithin(z) ||
                direction_ * z_error <= 0.0)
                stopAndObserve();
            else
            {
                geometry_msgs::Twist cmd;
                cmd.linear.x = boundedAlignmentSpeed(0.9 * z_error, linear_min_,
                                                       0.10);
                pub_.publish(cmd);
            }
            return Result::Waiting;
        }

        pub_.publish(geometry_msgs::Twist());
        if (stopped_at_.isZero()) resetMotion();
        const ros::Time settled_stamp = std::max(stopped_stamp_, last_motion_stamp_) +
                                        ros::Duration(settle_seconds_);
        const bool stopped = (now - stopped_at_).toSec() >= settle_seconds_ &&
                             (now - last_motion_).toSec() >= settle_seconds_;
        const bool new_still_frame = frame_stamp > settled_stamp &&
                                    (pose_stamp.isZero() || pose_stamp > settled_stamp);
        if (!stopped || !new_still_frame)
        {
            clearObservations();
            return Result::Waiting;
        }
        // 旧图像/旧检测不能凑成三帧；同一图像的重复控制循环也不增加计数。
        if ((!last_settled_frame_.isZero() && frame_stamp < last_settled_frame_) ||
            (frame_stamp != last_settled_frame_ && !pose_stamp.isZero() &&
             !last_settled_pose_.isZero() && pose_stamp <= last_settled_pose_))
        {
            clearObservations();
            return Result::Waiting;
        }
        if (confirmed_valid_)
        {
            ConfirmedObservation existing;
            if (frame == confirmed_.coordinate_frame && std::fabs(x) <= tolerance() &&
                depthWithin(z) && getConfirmedObservation(existing)) return Result::Ready;
            clearObservations();
        }
        if (last_settled_frame_ != frame_stamp)
        {
            const int sign = x > 0.0 ? 1 : x < 0.0 ? -1 : 0;
            const bool lateral_ok = std::fabs(x) <= tolerance();
            const bool depth_ok = depthWithin(z);
            // A reversal needs three consistent observations after stopping.
            if (settled_frames_ > 0 &&
                (frame != settled_frame_ || lateral_ok != settled_lateral_ok_ ||
                 depth_ok != settled_depth_ok_ ||
                 (sign != settled_sign_ && std::fabs(x) > tolerance())))
                clearObservations();
            if (settled_frames_ == 0)
            {
                sum_x_ = sum_z_ = 0.0;
                max_abs_x_ = 0.0;
                all_depth_ok_ = true;
                first_settled_frame_ = frame_stamp;
                first_settled_pose_ = pose_stamp;
                min_settled_z_ = max_settled_z_ = z;
                settled_sign_ = sign;
                settled_frame_ = frame;
                settled_lateral_ok_ = lateral_ok;
                settled_depth_ok_ = depth_ok;
            }
            last_settled_frame_ = frame_stamp;
            last_settled_pose_ = pose_stamp;
            min_settled_z_ = std::min(min_settled_z_, z);
            max_settled_z_ = std::max(max_settled_z_, z);
            sum_x_ += x;
            sum_z_ += z;
            max_abs_x_ = std::max(max_abs_x_, std::fabs(x));
            all_depth_ok_ = all_depth_ok_ && depthWithin(z);
            ++settled_frames_;
        }
        if (settled_frames_ < 3) return Result::Waiting;
        const double measured_x = sum_x_ / settled_frames_;
        const double measured_z = sum_z_ / settled_frames_;
        if (max_abs_x_ <= tolerance() && all_depth_ok_)
        {
            confirmed_.mean_x = measured_x;
            confirmed_.mean_z = measured_z;
            confirmed_.max_abs_x = max_abs_x_;
            confirmed_.min_z = min_settled_z_;
            confirmed_.max_z = max_settled_z_;
            confirmed_.frames = settled_frames_;
            confirmed_.first_frame_stamp = first_settled_frame_;
            confirmed_.last_frame_stamp = last_settled_frame_;
            confirmed_.first_pose_stamp = first_settled_pose_;
            confirmed_.last_pose_stamp = last_settled_pose_;
            confirmed_.coordinate_frame = settled_frame_;
            confirmed_valid_ = true;
            ConfirmedObservation checked;
            if (!getConfirmedObservation(checked)) { clearObservations(); return Result::Waiting; }
            ROS_INFO("AprilTag shared confirmation ready: frames=%u, mean_x=%+.1f mm, mean_z=%.1f mm, max_x=%.1f mm, first=%.9f, last=%.9f",
                     checked.frames, checked.mean_x * 1000.0, checked.mean_z * 1000.0,
                     checked.max_abs_x * 1000.0, checked.first_frame_stamp.toSec(), checked.last_frame_stamp.toSec());
            return Result::Ready;
        }

        if (max_abs_x_ > tolerance())
        {
            if (std::fabs(measured_x) <= tolerance())
            {
                resetMotion();
                return Result::Waiting;
            }
            if (std::fabs(measured_x) > coarse_threshold_)
            {
                state_ = State::Coarse;
                target_turn_ = std::max(0.005, 0.7 *
                    (std::atan2(std::fabs(measured_x), measured_z) -
                     std::atan2(coarse_threshold_ * 0.75, measured_z)));
                beginMotion(measured_x, frame);
                ROS_INFO("AprilTag coarse turn: x=%+.1f mm, speed=%.3f rad/s, odom_limit=%.2f deg",
                         measured_x * 1000.0, turn_speed_, target_turn_ * 180.0 / M_PI);
                publishTurn();
                return Result::Waiting;
            }
            if (pulses_ >= max_pulses_)
            {
                if (max_abs_x_ > hard_tolerance_)
                    return fail(hard_tolerance_ > 0.02
                        ? "AprilTag fine adjustment exhausted outside 40 mm tolerance"
                        : "AprilTag fine adjustment exhausted outside 20 mm tolerance");
                if (!fallback_)
                {
                    fallback_ = true;
                    ROS_WARN("AprilTag fine adjustment exhausted; accepting only confirmed residual within %.0f mm",
                             hard_tolerance_ * 1000.0);
                }
                // Start another stopped observation window after relaxing tolerance.
                resetMotion();
                return Result::Waiting;
            }
            state_ = State::Pulse;
            ++pulses_;
            target_turn_ = std::min(turn_speed_ * pulse_seconds_, std::max(0.005,
                0.7 * (std::atan2(std::fabs(measured_x), measured_z) -
                       std::atan2(fine_tolerance_, measured_z))));
            beginMotion(measured_x, frame);
            ROS_INFO("AprilTag fine pulse %d/%d: x=%+.1f mm, speed=%.3f rad/s, max_moving_time=%.3f s, odom_limit=%.2f deg",
                     pulses_, max_pulses_, measured_x * 1000.0, turn_speed_,
                     pulse_seconds_, target_turn_ * 180.0 / M_PI);
            publishTurn();
            return Result::Waiting;
        }
        if (!all_depth_ok_)
        {
            if (depthWithin(measured_z))
            {
                resetMotion();
                return Result::Waiting;
            }
            state_ = State::Linear;
            started_ = now;
            start_yaw_ = yaw_;
            direction_ = measured_z > target_z_ ? 1.0 : -1.0;
            motion_frame_ = frame;
            geometry_msgs::Twist cmd;
            cmd.linear.x = boundedAlignmentSpeed(0.9 * (measured_z - target_z_),
                                                 linear_min_, 0.10);
            pub_.publish(cmd);
        }
        else resetMotion(); // mixed-sign outliers: observe again instead of choosing a direction.
        return Result::Waiting;
    }

private:
    void clearObservations()
    {
        settled_frames_ = 0;
        last_settled_frame_ = first_settled_frame_ = ros::Time();
        last_settled_pose_ = first_settled_pose_ = ros::Time();
        confirmed_valid_ = false;
        confirmed_ = ConfirmedObservation();
    }
    static bool stampIsFresh(const ros::Time &stamp)
    {
        const double age = (ros::Time::now() - stamp).toSec();
        return !stamp.isZero() && age >= -0.1 && age <= 0.5;
    }
    static double boundedAlignmentSpeed(double requested, double minimum, double maximum)
    { return std::copysign(std::min(maximum, std::max(minimum, std::fabs(requested))), requested); }
    enum class State { Observe, Coarse, Pulse, Linear };
    void beginMotion(double x, const std::string &frame)
    {
        started_ = ros::WallTime::now();
        turn_started_at_ = ros::WallTime();
        start_yaw_ = yaw_;
        direction_ = x > 0.0 ? -1.0 : 1.0;
        motion_frame_ = frame;
    }
    void publishTurn()
    {
        geometry_msgs::Twist cmd;
        cmd.angular.z = direction_ * turn_speed_;
        pub_.publish(cmd);
    }
    void stopAndObserve()
    {
        pub_.publish(geometry_msgs::Twist());
        resetMotion();
    }
    Result fail(const char *reason)
    {
        clearObservations();
        pub_.publish(geometry_msgs::Twist());
        ROS_ERROR("%s", reason);
        return Result::Failed;
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (msg->header.stamp.isZero() || !stampIsFresh(msg->header.stamp) ||
            (!odom_stamp_.isZero() && msg->header.stamp <= odom_stamp_)) return;
        const auto &q = msg->pose.pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        const double px = msg->pose.pose.position.x, py = msg->pose.pose.position.y;
        if (!std::isfinite(norm) || norm < 0.5 || norm > 1.5 ||
            !std::isfinite(px) || !std::isfinite(py)) return;
        const double yaw = tf2::getYaw(q);
        const double angular = msg->twist.twist.angular.z, linear = msg->twist.twist.linear.x;
        if (!std::isfinite(yaw) || !std::isfinite(angular) || !std::isfinite(linear)) return;
        const ros::WallTime now = ros::WallTime::now();
        if (odom_received_.isZero() || std::fabs(angular) > 0.03 || std::fabs(linear) > 0.015 ||
            std::hypot(px - anchor_x_, py - anchor_y_) > 0.0015 ||
            std::fabs(std::atan2(std::sin(yaw - anchor_yaw_), std::cos(yaw - anchor_yaw_))) > 0.002)
        {
            // 一旦反馈显示再次运动，立即丢弃此前静止时的帧，不能混入新窗口。
            clearObservations();
            last_motion_ = now;
            last_motion_stamp_ = msg->header.stamp;
            anchor_x_ = px; anchor_y_ = py; anchor_yaw_ = yaw;
        }
        yaw_ = yaw;
        odom_received_ = now;
        odom_stamp_ = msg->header.stamp;
    }
    ros::Publisher pub_;
    ros::Subscriber odom_sub_;
    State state_ = State::Observe;
    double turn_speed_, linear_min_, fine_tolerance_, coarse_threshold_, pulse_seconds_, settle_seconds_;
    double startup_timeout_;
    double target_z_, near_tolerance_, far_tolerance_, hard_tolerance_;
    int max_pulses_, pulses_ = 0;
    bool fallback_ = false, all_depth_ok_ = true;
    bool settled_lateral_ok_ = false, settled_depth_ok_ = false;
    double yaw_ = 0.0, start_yaw_ = 0.0, direction_ = 0.0, target_turn_ = 0.0;
    double anchor_x_ = 0.0, anchor_y_ = 0.0, anchor_yaw_ = 0.0;
    ros::WallTime odom_received_, last_motion_, started_, stopped_at_, turn_started_at_;
    ros::Time odom_stamp_, last_motion_stamp_, stopped_stamp_, last_settled_frame_;
    ros::Time first_settled_frame_, first_settled_pose_, last_settled_pose_;
    double min_settled_z_ = 0.0, max_settled_z_ = 0.0;
    ConfirmedObservation confirmed_;
    bool confirmed_valid_ = false;
    unsigned int settled_frames_ = 0;
    int settled_sign_ = 0;
    double sum_x_ = 0.0, sum_z_ = 0.0, max_abs_x_ = 0.0;
    std::string motion_frame_, settled_frame_;
};

}
#endif
