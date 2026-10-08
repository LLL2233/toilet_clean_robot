#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "clean_toilet_robot/arm_service_checks.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "clean_toilet_robot/tag_recovery.h"
#include "clean_toilet_robot/tag_alignment.h"
using clean_toilet_robot::TagAlignment;
#include "clean_toilet_robot/retreat_arm_overlap.h"
using clean_toilet_robot::RetreatArmOverlap;
#include <algorithm>
#include <cmath>
#include <csignal>

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) { stop_requested = 1; }
bool taskActive() { return ros::ok() && !stop_requested; }

bool waitSeconds(double second)
{
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(second);
    while (taskActive() && ros::WallTime::now() < deadline)
        ros::WallDuration(0.02).sleep();
    return taskActive();
}

void stopBase(ros::Publisher &pub)
{
    for (int i = 0; i < 3; ++i)
    {
        pub.publish(geometry_msgs::Twist());
        ros::WallDuration(0.02).sleep();
    }
}

// One monotonic speed ramp. Duration is planned from its velocity integral.
bool velocitySegment(ros::Publisher &pub, double start_speed, double end_speed, double duration, RetreatArmOverlap *arm_return = nullptr)
{
    if (duration <= 1e-9) return taskActive();
    const ros::WallTime started = ros::WallTime::now();
    while (taskActive())
    {
        const double elapsed = (ros::WallTime::now() - started).toSec();
        if (elapsed >= duration) break;
        if (arm_return && !arm_return->update()) { stopBase(pub); return false; }
        geometry_msgs::Twist cmd;
        cmd.linear.x = start_speed + (end_speed - start_speed) * elapsed / duration;
        pub.publish(cmd);
        ros::spinOnce();
        ros::WallDuration(std::min(0.02, duration - elapsed)).sleep();
    }
    geometry_msgs::Twist cmd;
    if (taskActive()) cmd.linear.x = end_speed;
    pub.publish(cmd);
    return taskActive();
}

double current_x = 0.0;
double current_z = 0.0;
bool target_valid = false;
unsigned int valid_tag_frames = 0;
ros::WallTime last_tag_received;
ros::Time last_frame_stamp;
ros::Time last_pose_stamp;
std::string current_tag_frame;
const double tag_timeout = 0.5;
const double alignment_timeout = 60.0;

// PID控制参数
const double Kp = 3;                    // 比例系数
const double target_x_tolerance = 0.04; // Large button: accept lateral error within 4 cm.

const double z_target_distance = 0.15;
const double target_z_tolerance = 0.02;

bool aim_to_tag = false;

int tag_id = 1;

bool stampIsFresh(const ros::Time &stamp)
{
    if (stamp.isZero()) return true;
    const double age = (ros::Time::now() - stamp).toSec();
    return age >= -0.1 && age <= tag_timeout;
}

bool targetIsFresh()
{
    return target_valid &&
           stampIsFresh(last_frame_stamp) && stampIsFresh(last_pose_stamp) &&
           (ros::WallTime::now() - last_tag_received).toSec() <= tag_timeout;
}

void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (msg->header.stamp.isZero() || !stampIsFresh(msg->header.stamp))
    {
        target_valid = false;
        valid_tag_frames = 0;
        return;
    }
    if (!last_frame_stamp.isZero() && msg->header.stamp <= last_frame_stamp) return;
    const bool previous_target_fresh = targetIsFresh();
    last_frame_stamp = msg->header.stamp;
    target_valid = false;

    for (const auto &detection : msg->detections)
    {
        if (detection.id.empty() || detection.id[0] != tag_id ||
            !stampIsFresh(detection.pose.header.stamp)) continue;
        const auto &position = detection.pose.pose.pose.position;
        if (!std::isfinite(position.x) || !std::isfinite(position.z) || position.z <= 0.0)
            continue;
        const std::string tag_frame = detection.pose.header.frame_id.empty()
            ? msg->header.frame_id : detection.pose.header.frame_id;
        if (!previous_target_fresh || tag_frame != current_tag_frame) valid_tag_frames = 0;
        current_tag_frame = tag_frame;
        current_x = position.x;
        current_z = position.z;
        last_pose_stamp = detection.pose.header.stamp;
        last_tag_received = ros::WallTime::now();
        target_valid = true;
        valid_tag_frames = std::min(valid_tag_frames + 1U, 3U);
        return;
    }
    valid_tag_frames = 0;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mgrab_test", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    ros::NodeHandle nh;
    clean_toilet_robot::ArmActionLock action_lock;
    if (!action_lock.ready()) return 1;
    clean_toilet_robot::ActionTiming action_timing("Button");
    ros::NodeHandle transport_nh;
    if (!clean_toilet_robot::requireArmTransport(transport_nh)) return 1;
    ros::NodeHandle private_nh_("~");

    private_nh_.param("tag", tag_id, 1);
    ROS_INFO(" The value of tag is %d .", tag_id);

    // On this base, manual testing found that rotation starts at 0.27 rad/s.
    double min_angular_speed = 0.30;
    double min_linear_speed = 0.04;
    private_nh_.param("alignment_min_angular_speed", min_angular_speed, min_angular_speed);
    private_nh_.param("alignment_min_linear_speed", min_linear_speed, min_linear_speed);
    if (!std::isfinite(min_angular_speed) || min_angular_speed < 0.28 || min_angular_speed > 0.5 ||
        !std::isfinite(min_linear_speed) || min_linear_speed < 0.0 || min_linear_speed > 0.1)
    {
        ROS_ERROR("Invalid AprilTag alignment minimum speeds: angular must be within [0.28, 0.5] rad/s, linear within [0, 0.1] m/s");
        return 1;
    }
    ROS_INFO("AprilTag alignment minimum speeds: angular=%.3f rad/s, linear=%.3f m/s",
             min_angular_speed, min_linear_speed);

    double fine_tolerance = 0.04, coarse_threshold = 0.06;
    double pulse_seconds = 0.10, settle_seconds = 0.20;
    double startup_timeout = 0.60;
    int max_pulses = 6;
    std::string alignment_odom_topic = "/odom";
    private_nh_.param("alignment_fine_tolerance_m", fine_tolerance, fine_tolerance);
    private_nh_.param("alignment_coarse_threshold_m", coarse_threshold, coarse_threshold);
    private_nh_.param("alignment_pulse_sec", pulse_seconds, pulse_seconds);
    private_nh_.param("alignment_settle_sec", settle_seconds, settle_seconds);
    private_nh_.param("alignment_startup_timeout_sec", startup_timeout, startup_timeout);
    private_nh_.param("alignment_max_pulses", max_pulses, max_pulses);
    private_nh_.param("recovery_odom_topic", alignment_odom_topic, alignment_odom_topic);
    if (!std::isfinite(fine_tolerance) || fine_tolerance < 0.005 || fine_tolerance > target_x_tolerance ||
        !std::isfinite(coarse_threshold) || coarse_threshold <= target_x_tolerance || coarse_threshold > 0.15 ||
        !std::isfinite(pulse_seconds) || pulse_seconds < 0.06 || pulse_seconds > 0.25 ||
        !std::isfinite(settle_seconds) || settle_seconds < 0.10 || settle_seconds > 0.80 ||
        !std::isfinite(startup_timeout) || startup_timeout < 0.15 || startup_timeout > 1.0 ||
        max_pulses < 1 || max_pulses > 12 || alignment_odom_topic.empty())
    {
        ROS_ERROR("Invalid AprilTag pulse alignment parameters; task not started");
        return 1;
    }
    ROS_INFO("AprilTag pulse alignment: fine=%.0f mm, coarse=%.0f mm, pulse=%.3f s, settle=%.3f s, startup_timeout=%.3f s, max_pulses=%d",
             fine_tolerance * 1000.0, coarse_threshold * 1000.0,
             pulse_seconds, settle_seconds, startup_timeout, max_pulses);

    double approach_distance = 0.07;  // Preserve the previous nominal 7 cm approach.
    double approach_speed = 0.08;
    double contact_distance = 0.02;   // Final 2 cm includes the deceleration to zero.
    double contact_speed = 0.04;
    double acceleration = 0.20;
    double retreat_distance = 0.20;   // Previous 20 ticks at 0.1 m/s was 20 cm.
    double retreat_speed = 0.20;
    private_nh_.param("button_approach_distance", approach_distance, approach_distance);
    private_nh_.param("button_approach_speed", approach_speed, approach_speed);
    private_nh_.param("button_contact_distance", contact_distance, contact_distance);
    private_nh_.param("button_contact_speed", contact_speed, contact_speed);
    private_nh_.param("button_acceleration", acceleration, acceleration);
    private_nh_.param("button_retreat_distance", retreat_distance, retreat_distance);
    private_nh_.param("button_retreat_speed", retreat_speed, retreat_speed);
    if (!std::isfinite(approach_distance) || approach_distance <= 0.0 || approach_distance > 0.15 ||
        !std::isfinite(approach_speed) || approach_speed < 0.04 || approach_speed > 0.10 ||
        !std::isfinite(contact_distance) || contact_distance <= 0.0 || contact_distance > approach_distance ||
        !std::isfinite(contact_speed) || contact_speed < 0.04 || contact_speed > approach_speed ||
        !std::isfinite(acceleration) || acceleration < 0.05 || acceleration > 0.50 ||
        !std::isfinite(retreat_distance) || retreat_distance <= 0.0 || retreat_distance > 0.50 ||
        !std::isfinite(retreat_speed) || retreat_speed < 0.04 || retreat_speed > 0.20)
    {
        ROS_ERROR("Invalid button motion parameters; task not started");
        return 1;
    }
    // Camera-Z residual is frozen after three fresh, aligned observations.
    // Shorten only: a farther tag must not cause a deeper button press.
    double button_depth_reference_m = z_target_distance;
    double button_depth_compensation_gain = 1.0;
    double button_depth_max_shortening_m = 0.02;
    private_nh_.param("button_depth_reference_m", button_depth_reference_m, button_depth_reference_m);
    private_nh_.param("button_depth_compensation_gain", button_depth_compensation_gain, button_depth_compensation_gain);
    private_nh_.param("button_depth_max_shortening_m", button_depth_max_shortening_m, button_depth_max_shortening_m);
    if (!std::isfinite(button_depth_reference_m) || button_depth_reference_m < 0.10 || button_depth_reference_m > 0.30 ||
        !std::isfinite(button_depth_compensation_gain) || button_depth_compensation_gain < 0.0 || button_depth_compensation_gain > 3.0 ||
        !std::isfinite(button_depth_max_shortening_m) || button_depth_max_shortening_m < 0.0 || button_depth_max_shortening_m > 0.03)
    {
        ROS_ERROR("Invalid button depth compensation: reference in [0.10, 0.30] m, gain in [0, 3], max shortening in [0, 0.03] m");
        return 1;
    }
    ROS_INFO("Button depth settings: reference_z=%.1f mm, gain=%.2f, max_shortening=%.1f mm (shorten only)",
             button_depth_reference_m * 1000.0, button_depth_compensation_gain, button_depth_max_shortening_m * 1000.0);

    const double accelerate_time = approach_speed / acceleration;
    const double slow_time = (approach_speed - contact_speed) / acceleration;
    const double stop_time = contact_speed / acceleration;
    const double accelerate_distance = 0.5 * approach_speed * accelerate_time;
    const double slow_distance = 0.5 * (approach_speed + contact_speed) * slow_time;
    const double stop_distance = 0.5 * contact_speed * stop_time;
    double cruise_distance = approach_distance - accelerate_distance - slow_distance - contact_distance;
    const double contact_cruise_distance = contact_distance - stop_distance;
    const double retreat_ramp_time = retreat_speed / acceleration;
    const double retreat_cruise_distance = retreat_distance - retreat_speed * retreat_ramp_time;
    if (cruise_distance < -1e-9 || contact_cruise_distance < -1e-9 || retreat_cruise_distance < -1e-9)
    {
        ROS_ERROR("Button distances too short for the configured speed ramps; task not started");
        return 1;
    }

    RetreatArmOverlap::Settings retreat_arm_settings;
    // 按按钮后先后退 0.5 秒，再异步回零，与剩余后退重叠。
    retreat_arm_settings.start_delay = 0.5;
    if (!RetreatArmOverlap::readSettings(private_nh_, (2.0 * retreat_ramp_time + std::max(0.0, retreat_cruise_distance) / retreat_speed), 1.0, retreat_arm_settings)) return 1;

    // 使用到的 service
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_hor_service_close");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber tag_sub_ = nh.subscribe("tag_detections", 1, tagCallback);
    TagRecovery recovery(nh, private_nh_, pub);

    ros::WallRate loop_rate(10); // Preserve task approach/retreat timing.
    ros::WallRate alignment_rate(50);
    TagAlignment alignment(nh, pub, min_angular_speed, min_linear_speed, fine_tolerance,
                           coarse_threshold, pulse_seconds, settle_seconds, startup_timeout, max_pulses,
                           alignment_odom_topic, z_target_distance, target_z_tolerance, target_z_tolerance,
                           target_x_tolerance);

    geometry_msgs::Twist vel_msg;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    //首先，对齐到按键下方的ApriTag码
    const ros::WallTime alignment_started = ros::WallTime::now();
    while (taskActive() && !aim_to_tag)
    {
        ros::spinOnce();
        if (!taskActive()) { stopBase(pub); return 1; }
        if ((ros::WallTime::now() - alignment_started).toSec() >= alignment_timeout)
        {
            stopBase(pub);
            ROS_ERROR("AprilTag alignment timed out after %.0f seconds", alignment_timeout);
            return 1;
        }
        const bool within_tolerance = std::fabs(current_x) <= alignment.tolerance() && alignment.depthWithin(current_z);
        const TagRecovery::Result recovered = recovery.update(targetIsFresh(), valid_tag_frames, within_tolerance, true);
        if (recovered == TagRecovery::Result::Failed) { stopBase(pub); return 1; }
        if (recovered != TagRecovery::Result::Ready)
        {
            alignment.resetMotion();
            alignment_rate.sleep();
            continue;
        }
        const TagAlignment::Result adjustment = alignment.update(
            current_x, current_z, last_frame_stamp, last_pose_stamp, current_tag_frame);
        if (adjustment == TagAlignment::Result::Failed) { stopBase(pub); return 1; }
        if (adjustment != TagAlignment::Result::Ready || !within_tolerance)
        {
            alignment_rate.sleep();
            continue;
        }
        // 直接复用停稳后三帧，完成判定与补偿使用同一组数据。
        TagAlignment::ConfirmedObservation confirmed;
        if (!targetIsFresh() || !alignment.getConfirmedObservation(confirmed))
        {
            stopBase(pub);
            alignment.resetMotion();
            alignment_rate.sleep();
            continue;
        }
        {
            pub.publish(geometry_msgs::Twist());
            ROS_INFO("AprilTag alignment complete after 3 distinct aligned frames: x_error=%.1f mm, z_error=%.1f mm",
                     confirmed.mean_x * 1000.0, (confirmed.mean_z - z_target_distance) * 1000.0);
            const double saved_camera_z = confirmed.mean_z;
            if (!targetIsFresh() || !std::isfinite(saved_camera_z) || saved_camera_z <= 0.0)
            {
                stopBase(pub);
                ROS_ERROR("Invalid or stale button depth samples; no press attempted");
                return 1;
            }
            const double z_error = saved_camera_z - button_depth_reference_m;
            const double requested_shortening = std::max(0.0, -z_error) * button_depth_compensation_gain;
            // Keep every speed ramp and the complete slow-contact phase.
            const double available_shortening = std::max(0.0, cruise_distance);
            const double applied_shortening = std::min(requested_shortening,
                std::min(button_depth_max_shortening_m, available_shortening));
            approach_distance -= applied_shortening;
            cruise_distance = std::max(0.0, cruise_distance - applied_shortening);
            ROS_INFO("Saved button depth residual: camera_z=%.1f mm (%u frames), reference_z=%.1f mm, z_error=%+.1f mm, requested_shortening=%.1f mm, applied_shortening=%.1f mm, approach_distance=%.1f mm",
                     saved_camera_z * 1000.0, confirmed.frames, button_depth_reference_m * 1000.0,
                     z_error * 1000.0, requested_shortening * 1000.0, applied_shortening * 1000.0,
                     approach_distance * 1000.0);
            if (requested_shortening > applied_shortening + 1e-9)
                ROS_INFO("Button depth shortening limited by configured cap or available cruise distance");
            aim_to_tag = true;
            break;
        }
        alignment_rate.sleep();
    }
    pub.publish(geometry_msgs::Twist());
    if (!taskActive() || !aim_to_tag) { stopBase(pub); return 1; }

    ROS_INFO("tf coordinate transformaing....");

    // 获取tag到机械臂基坐标的坐标变换
    geometry_msgs::TransformStamped tfs_1;
    try
    {
        tfs_1 = buffer.lookupTransform("arm_base_link", "tag_" + std::to_string(tag_id),
                                      ros::Time(0), ros::Duration(0.5));
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_ERROR("Fresh tag transform unavailable: %s", ex.what());
        return 1;
    }
    if (tfs_1.header.stamp.isZero() || !stampIsFresh(tfs_1.header.stamp))
    {
        ROS_ERROR("Tag transform is stale; arm motion cancelled");
        return 1;
    }

    // 单位转换，ros坐标系到逆运算坐标系
    int x = -int(tfs_1.transform.translation.y * 1000);
    int y = int(tfs_1.transform.translation.x * 1000 + 0);
    int z = int(tfs_1.transform.translation.z * 1000 + 200);
    ROS_INFO("Target X: %d, Target Y: %d, Target Z: %d", x, y, z);

    // 第一步，平端运动到与按键水平
    upros_message::ArmPosition move_srv;
    move_srv.request.x = x-40;
    move_srv.request.y = 250.0;
    move_srv.request.z = z+30;
    if (!taskActive()) { stopBase(pub); return 1; }
    ros::spinOnce();
    TagAlignment::ConfirmedObservation before_arm;
    std::string shared_confirmation_reason;
    const bool shared_confirmation_valid = alignment.getConfirmedObservation(before_arm, &shared_confirmation_reason);
    if (!targetIsFresh() || !shared_confirmation_valid ||
        current_tag_frame != before_arm.coordinate_frame ||
        std::fabs(current_x) > alignment.tolerance() || !alignment.depthWithin(current_z))
    {
        stopBase(pub);
        ROS_ERROR("AprilTag confirmation blocked before task motion: reason=%s, tag_fresh=%d, image_age=%.3f s, current_x=%+.1f mm, current_z=%.1f mm, observed_frame=%s, confirmed_frame=%s; task stopped",
                  shared_confirmation_reason.empty() ? "current tag frame or range changed" : shared_confirmation_reason.c_str(),
                  targetIsFresh(), (ros::Time::now() - last_frame_stamp).toSec(),
                  current_x * 1000.0, current_z * 1000.0,
                  current_tag_frame.c_str(), before_arm.coordinate_frame.c_str());
        return 1;
    }
    if (!clean_toilet_robot::callArmPosition(arm_move_close_client, move_srv, "Button position"))
    {
        stopBase(pub);
        ROS_ERROR("Button arm positioning service failed; no press attempted");
        return 1;
    }
    if (!waitSeconds(1.0)) { stopBase(pub); return 1; }

    // Single approach: accelerate, slow for contact, stop gently, then retreat.
    // No dwell against the button and no automatic second press.
    ROS_INFO("Button single press: approach=%.1f mm, cruise=%.3f m/s, final_contact=%.1f mm at %.3f m/s, acceleration=%.3f m/s^2",
             approach_distance * 1000.0, approach_speed, contact_distance * 1000.0, contact_speed, acceleration);
    if (!velocitySegment(pub, 0.0, approach_speed, accelerate_time) ||
        !velocitySegment(pub, approach_speed, approach_speed, std::max(0.0, cruise_distance) / approach_speed) ||
        !velocitySegment(pub, approach_speed, contact_speed, slow_time) ||
        !velocitySegment(pub, contact_speed, contact_speed, std::max(0.0, contact_cruise_distance) / contact_speed) ||
        !velocitySegment(pub, contact_speed, 0.0, stop_time))
    {
        stopBase(pub);
        ROS_ERROR("Button approach interrupted; no retry");
        return 1;
    }

    // Immediately release the button; overlap homing with the same smooth retreat.
    RetreatArmOverlap arm_return(pub, retreat_arm_settings, "Button", [arm_zero_client]() mutable {
        std_srvs::Empty srv;
        return clean_toilet_robot::callArmEmpty(arm_zero_client, srv, "Button home");
    }, taskActive);
    ROS_INFO("Button press complete; retreating immediately %.1f mm", retreat_distance * 1000.0);
    if (!velocitySegment(pub, 0.0, -retreat_speed, retreat_ramp_time, &arm_return) ||
        !velocitySegment(pub, -retreat_speed, -retreat_speed, std::max(0.0, retreat_cruise_distance) / retreat_speed, &arm_return) ||
        !velocitySegment(pub, -retreat_speed, 0.0, retreat_ramp_time, &arm_return))
    {
        stopBase(pub);
        arm_return.cancel();
        ROS_ERROR("Button retreat interrupted or arm return failed");
        return 1;
    }
    stopBase(pub);
    if (!taskActive() || !arm_return.finish()) { arm_return.cancel(); return 1; }

    action_timing.complete();
    ros::shutdown();

    return 0;
}
