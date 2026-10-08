#ifndef CLEAN_TOILET_ROBOT_RETREAT_ARM_OVERLAP_H
#define CLEAN_TOILET_ROBOT_RETREAT_ARM_OVERLAP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <string>

namespace clean_toilet_robot {

// One arm command after initial retreat clearance; service latency never blocks
// the caller's velocity loop. Like hover, settling starts after acknowledgement.
class RetreatArmOverlap {
public:
    struct Settings { double start_delay = 1.5; double wait = 0.5; };
    static bool readSettings(ros::NodeHandle &nh, double retreat_seconds,
                             double default_wait, Settings &settings) {
        settings.wait = default_wait;
        nh.param("retreat_arm_start_delay_sec", settings.start_delay, settings.start_delay);
        nh.param("retreat_arm_wait_sec", settings.wait, settings.wait);
        if (!std::isfinite(settings.start_delay) || settings.start_delay < 0.1 ||
            settings.start_delay > retreat_seconds - 0.2 ||
            !std::isfinite(settings.wait) || settings.wait < 0.3 || settings.wait > 5.0) {
            ROS_ERROR("Invalid retreat/arm overlap: start delay in [0.1, %.2f] s; arm wait in [0.3, 5.0] s",
                      retreat_seconds - 0.2);
            return false;
        }
        ROS_INFO("Retreat/arm settings: planned_retreat=%.2f s, start_delay=%.2f s, wait=%.2f s",
                 retreat_seconds, settings.start_delay, settings.wait);
        return true;
    }
    RetreatArmOverlap(ros::Publisher pub, Settings settings, std::string label,
                      std::function<bool()> command, std::function<bool()> active)
        : pub_(pub), settings_(settings), label_(label), command_(command), active_(active),
          started_(ros::WallTime::now()) {}
    ~RetreatArmOverlap() {
        if (call_.valid()) cancel();
    }
    bool update() {
        if (!active_()) return false;
        const double elapsed = (ros::WallTime::now() - started_).toSec();
        if (!requested_ && elapsed >= settings_.start_delay) {
            requested_ = true;
            requested_at_ = ros::WallTime::now();
            ROS_INFO("%s starting arm return during retreat: elapsed=%.2f s", label_.c_str(), elapsed);
            call_ = std::async(std::launch::async, command_);
        }
        if (!requested_ || accepted_) return true;
        if (call_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            if (!call_.get()) {
                ROS_ERROR("%s arm return service failed; retreat stopped", label_.c_str());
                return false;
            }
            accepted_ = true;
            deadline_ = ros::WallTime::now() + ros::WallDuration(settings_.wait);
            ROS_INFO("%s arm return accepted; %.2f s settling overlaps remaining retreat",
                     label_.c_str(), settings_.wait);
        } else if ((ros::WallTime::now() - requested_at_).toSec() >= 2.0) {
            ROS_ERROR("%s arm return response timed out; retreat stopped", label_.c_str());
            return false;
        }
        return true;
    }
    bool finish() {
        if (!requested_) { cancel(); return false; }
        while (active_()) {
            if (!update()) { cancel(); return false; }
            if (accepted_ && ros::WallTime::now() >= deadline_) {
                ROS_INFO("%s retreat and arm settling complete", label_.c_str());
                return true;
            }
            pub_.publish(geometry_msgs::Twist());
            ros::spinOnce();
            ros::WallDuration(0.05).sleep();
        }
        cancel();
        return false;
    }
    void cancel() {
        for (int i = 0; i < 3; ++i) {
            pub_.publish(geometry_msgs::Twist());
            ros::WallDuration(0.02).sleep();
        }
        // Abort an outstanding ROS request before waiting for its worker.
        if (call_.valid()) {
            ros::shutdown();
            call_.wait();
            call_ = std::future<bool>();
        }
    }
private:
    ros::Publisher pub_;
    Settings settings_;
    std::string label_;
    std::function<bool()> command_, active_;
    ros::WallTime started_, requested_at_, deadline_;
    std::future<bool> call_;
    bool requested_ = false, accepted_ = false;
};
} // namespace clean_toilet_robot
#endif
