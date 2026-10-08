#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "clean_toilet_robot/arm_service_checks.h"
#include "clean_toilet_robot/base_stop_confirmation.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "clean_toilet_robot/tag_recovery.h"
#include "clean_toilet_robot/tag_alignment.h"
using clean_toilet_robot::TagAlignment;
#include "clean_toilet_robot/retreat_arm_overlap.h"
using clean_toilet_robot::RetreatArmOverlap;
#include <algorithm>
#include <csignal>
#include <cmath>
#include <string>
#include <geometry_msgs/Vector3Stamped.h>

volatile std::sig_atomic_t retreat_stop_requested = 0;
void request_retreat_stop(int) { retreat_stop_requested = 1; }

void sleep(double second)
{
    const auto started = ros::WallTime::now();
    ros::Duration(second).sleep();
    ROS_INFO("Task wait: planned=%.3f s, actual=%.3f s", second,
             (ros::WallTime::now() - started).toSec());
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
const double target_x_tolerance = 0.02; // X轴位置容忍误差

const double z_target_distance = 0.20;
const double target_z_tolerance = 0.02;
const double max_grab_lateral_compensation_mm = 20.0;
const double nominal_grab_approach_distance = 0.14; // metres
const double grab_approach_speed = 0.10; // metres/second
const double max_grab_depth_compensation = 0.03; // metres

bool aim_to_tag = false;

int tag_id = 1;

double boundedAlignmentSpeed(double requested, double minimum, double maximum)
{
    return std::copysign(std::min(maximum, std::max(minimum, std::fabs(requested))), requested);
}

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

    ros::init(argc, argv, "mgrab_test");
    ros::NodeHandle nh;
    clean_toilet_robot::ArmActionLock action_lock;
    if (!action_lock.ready()) return 1;
    clean_toilet_robot::ActionTiming action_timing("Grab");
    ros::NodeHandle transport_nh;
    if (!clean_toilet_robot::requireArmTransport(transport_nh)) return 1;
    ros::NodeHandle private_nh_("~");

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

    // Be stricter when the robot is too close to the tag.
    double near_z_tolerance = 0.01;
    double approach_far_z_gain = 1.0;
    double approach_near_z_gain = 1.2;
    private_nh_.param("alignment_near_z_tolerance", near_z_tolerance, near_z_tolerance);
    private_nh_.param("approach_far_z_gain", approach_far_z_gain, approach_far_z_gain);
    private_nh_.param("approach_near_z_gain", approach_near_z_gain, approach_near_z_gain);
    if (!std::isfinite(near_z_tolerance) || near_z_tolerance <= 0.0 || near_z_tolerance > target_z_tolerance ||
        !std::isfinite(approach_far_z_gain) || approach_far_z_gain < 0.0 || approach_far_z_gain > 3.0 ||
        !std::isfinite(approach_near_z_gain) || approach_near_z_gain < 0.0 || approach_near_z_gain > 3.0)
    {
        ROS_ERROR("Invalid grab depth settings: near tolerance must be within (0, %.3f] m; gains within [0, 3]", target_z_tolerance);
        return 1;
    }
    ROS_INFO("Grab depth settings: accepted camera_z=%.0f..%.0f mm, far_gain=%.2f, near_gain=%.2f",
             (z_target_distance - near_z_tolerance) * 1000.0,
             (z_target_distance + target_z_tolerance) * 1000.0, approach_far_z_gain, approach_near_z_gain);

    double fine_tolerance = 0.02, coarse_threshold = 0.04;
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
    if (!std::isfinite(fine_tolerance) || fine_tolerance < 0.005 || fine_tolerance > 0.02 ||
        !std::isfinite(coarse_threshold) || coarse_threshold <= 0.02 || coarse_threshold > 0.15 ||
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

    const int retreat_ticks = 18;
    const double retreat_frequency_hz = 10.0;
    const double retreat_seconds = retreat_ticks / retreat_frequency_hz;
    RetreatArmOverlap::Settings retreat_arm_settings;
    if (!RetreatArmOverlap::readSettings(private_nh_, retreat_seconds, 0.5, retreat_arm_settings)) return 1;

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_hor_service_open");
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_hor_service_close");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber tag_sub_ = nh.subscribe("tag_detections", 1, tagCallback);
    TagRecovery recovery(nh, private_nh_, pub);

    ros::WallRate loop_rate(10); // Preserve task approach/retreat timing.
    ros::WallRate alignment_rate(50);
    TagAlignment alignment(nh, pub, min_angular_speed, min_linear_speed, fine_tolerance,
                           coarse_threshold, pulse_seconds, settle_seconds, startup_timeout, max_pulses,
                           alignment_odom_topic, z_target_distance, near_z_tolerance, target_z_tolerance);

    geometry_msgs::Twist vel_msg;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    double grab_lateral_compensation_mm = 0.0;
    double grab_approach_distance = nominal_grab_approach_distance;

    // 对齐到下方的ApriTag码
    const ros::WallTime alignment_started = ros::WallTime::now();
    while (ros::ok() && !aim_to_tag)
    {
        ros::spinOnce();
        if (!ros::ok()) { pub.publish(geometry_msgs::Twist()); return 1; }
        geometry_msgs::Twist cmd;
        const bool tag_fresh = targetIsFresh();
        const bool within_tolerance =
            std::fabs(current_x) <= alignment.tolerance() && alignment.depthWithin(current_z);
        if ((ros::WallTime::now() - alignment_started).toSec() >= alignment_timeout)
        {
            pub.publish(cmd);
            ROS_ERROR("AprilTag alignment timed out after %.0f seconds: fresh=%d, x_error=%.1f mm, z_error=%.1f mm",
                      alignment_timeout, tag_fresh, current_x * 1000.0,
                      (current_z - z_target_distance) * 1000.0);
            return 1;
        }
        const TagRecovery::Result recovery_result =
            recovery.update(tag_fresh, valid_tag_frames, within_tolerance, true);
        if (recovery_result == TagRecovery::Result::Failed)
        {
            pub.publish(geometry_msgs::Twist());
            return 1;
        }
        if (recovery_result != TagRecovery::Result::Ready)
        {
            alignment.resetMotion();
            ROS_WARN_THROTTLE(2.0, "Waiting for AprilTag %d: fresh=%d, valid_frames=%u; motion waits for stable observations or bounded tag-loss recovery",
                              tag_id, tag_fresh, valid_tag_frames);
            alignment_rate.sleep();
            continue;
        }
        const TagAlignment::Result adjustment = alignment.update(
            current_x, current_z, last_frame_stamp, last_pose_stamp, current_tag_frame);
        if (adjustment == TagAlignment::Result::Failed)
        {
            pub.publish(geometry_msgs::Twist());
            return 1;
        }
        if (adjustment != TagAlignment::Result::Ready)
        {
            alignment_rate.sleep();
            continue;
        }
        // 直接复用停稳后三帧，完成判定与补偿使用同一组数据。
        TagAlignment::ConfirmedObservation confirmed;
        if (!targetIsFresh() || !alignment.getConfirmedObservation(confirmed))
        {
            pub.publish(geometry_msgs::Twist());
            alignment.resetMotion();
            alignment_rate.sleep();
            continue;
        }
        if (std::fabs(current_x) <= alignment.tolerance() && alignment.depthWithin(current_z))
        {
            pub.publish(cmd);
            // 这一组已由共享对准模块确认，不再等待另外三帧。
            const double saved_horizontal_error = confirmed.mean_x;
            if (confirmed.coordinate_frame.empty())
            {
                ROS_ERROR("Cannot save grab compensation without a tag coordinate frame; grabbing cancelled");
                return 1;
            }
            // Transform a displacement, so camera mounting translations
            // do not get added to the measured residual alignment error.
            geometry_msgs::Vector3Stamped camera_offset;
            camera_offset.header.frame_id = confirmed.coordinate_frame;
            camera_offset.header.stamp = confirmed.last_pose_stamp.isZero() ? confirmed.last_frame_stamp : confirmed.last_pose_stamp;
            camera_offset.vector.x = saved_horizontal_error;
            geometry_msgs::Vector3Stamped arm_offset;
            try
            {
                arm_offset = buffer.transform(camera_offset, "arm_base_link", ros::Duration(0.5));
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_ERROR("Cannot transform saved grab horizontal error; grabbing cancelled: %s", ex.what());
                return 1;
            }
            if (!std::isfinite(arm_offset.vector.y))
            {
                ROS_ERROR("Non-finite grab compensation; grabbing cancelled");
                return 1;
            }
            // ArmPosition uses the same x=-arm_base_link.y mapping as arm_put,
            // with millimetres; the arm solver ultimately uses integer mm.
            // AprilTag x describes the TAG relative to the camera, not the
            // robot's displacement: robot-right means tag-left (negative x).
            // Move the endpoint toward the tag, opposite the robot's bias.
            const double requested_offset_mm = -arm_offset.vector.y * 1000.0;
            grab_lateral_compensation_mm = std::round(std::max(-max_grab_lateral_compensation_mm,
                std::min(max_grab_lateral_compensation_mm, requested_offset_mm)));
            ROS_INFO("Saved grab horizontal residual: camera_x=%+.1f mm (%u frames), arm_x_compensation=%+.0f mm",
                     saved_horizontal_error * 1000.0, confirmed.frames, grab_lateral_compensation_mm);
            // Freeze the same three accepted depth samples before moving the arm/base.
            const double saved_camera_z = confirmed.mean_z;
            const double saved_z_error = saved_camera_z - z_target_distance;
            const double depth_gain = saved_z_error < 0.0 ? approach_near_z_gain : approach_far_z_gain;
            const double depth_compensation = std::max(-max_grab_depth_compensation,
                std::min(max_grab_depth_compensation, saved_z_error * depth_gain));
            grab_approach_distance = nominal_grab_approach_distance + depth_compensation;
            ROS_INFO("Saved grab depth residual: camera_z=%.1f mm (%u frames), z_error=%+.1f mm, gain=%.2f, compensation=%+.1f mm, approach_distance=%.1f mm, approach_duration=%.3f s",
                     saved_camera_z * 1000.0, confirmed.frames, saved_z_error * 1000.0, depth_gain,
                     depth_compensation * 1000.0, grab_approach_distance * 1000.0, grab_approach_distance / grab_approach_speed);
            ROS_INFO("AprilTag alignment complete after 3 distinct aligned frames: x_error=%.1f mm, z_error=%.1f mm",
                     confirmed.mean_x * 1000.0, (confirmed.mean_z - z_target_distance) * 1000.0);
            aim_to_tag = true;
            break;
        }
        alignment_rate.sleep();
    }
    pub.publish(geometry_msgs::Twist());
    if (!aim_to_tag) return 1;

    ROS_INFO("tf coordinate transformaing....");

    // 获取tag到机械臂基坐标的坐标变换
    geometry_msgs::TransformStamped tfs_1;
    try
    {
        tfs_1 = buffer.lookupTransform("arm_base_link", "tag_1", ros::Time(0),
                                      ros::Duration(0.5));
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
    int y = int(tfs_1.transform.translation.x * 1000);
    int z = int(tfs_1.transform.translation.z * 1000);

    ROS_INFO("Target X: %d, Target Y: %d, Target Z: %d", x, y, z);
    std_srvs::Empty empty_srv;

    // 运动到抓取位置
    upros_message::ArmPosition move_srv;
    move_srv.request.x = grab_lateral_compensation_mm;
    move_srv.request.y = 300;
    move_srv.request.z = 40;
    ROS_INFO("Grab target: x=%.0f mm, y=%.0f mm, z=%.0f mm (saved lateral compensation)",
             move_srv.request.x, move_srv.request.y, move_srv.request.z);
    ros::spinOnce();
    TagAlignment::ConfirmedObservation before_arm;
    std::string shared_confirmation_reason;
    const bool shared_confirmation_valid = alignment.getConfirmedObservation(before_arm, &shared_confirmation_reason);
    if (!targetIsFresh() || !shared_confirmation_valid ||
        current_tag_frame != before_arm.coordinate_frame ||
        std::fabs(current_x) > alignment.tolerance() || !alignment.depthWithin(current_z))
    {
        pub.publish(geometry_msgs::Twist());
        ROS_ERROR("AprilTag confirmation blocked before task motion: reason=%s, tag_fresh=%d, image_age=%.3f s, current_x=%+.1f mm, current_z=%.1f mm, observed_frame=%s, confirmed_frame=%s; task stopped",
                  shared_confirmation_reason.empty() ? "current tag frame or range changed" : shared_confirmation_reason.c_str(),
                  targetIsFresh(), (ros::Time::now() - last_frame_stamp).toSec(),
                  current_x * 1000.0, current_z * 1000.0,
                  current_tag_frame.c_str(), before_arm.coordinate_frame.c_str());
        return 1;
    }
    if (!clean_toilet_robot::callArmPosition(arm_move_open_client, move_srv, "Grab position")) { pub.publish(geometry_msgs::Twist()); return 1; }
    sleep(1.0);

    // 打开夹爪
    // arm_release_client.call(empty_srv);
    // sleep(1.0);

    // Adjust the nominal 14 cm approach using the saved camera-Z residual.
    // Too close: travel less, with a stronger reduction than the far-side correction.
    vel_msg.linear.x = grab_approach_speed;
    const double approach_duration = grab_approach_distance / grab_approach_speed;
    const ros::WallTime approach_deadline = ros::WallTime::now() + ros::WallDuration(approach_duration);
    while (ros::ok())
    {
        const double remaining = (approach_deadline - ros::WallTime::now()).toSec();
        if (remaining <= 0.0) break;
        pub.publish(vel_msg);
        ros::spinOnce();
        // Finer timing avoids rounding compensation to the old 10 mm control steps.
        ros::WallDuration(std::min(0.02, remaining)).sleep();
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    if (!ros::ok()) return 1;
    clean_toilet_robot::BaseStopConfirmation grab_stop(nh,pub,alignment_odom_topic,0.3,0.2,1.7);
    if (!grab_stop.wait([](){return ros::ok();},"grab_before_close")) return 1;

    // 闭合夹爪
    if (!clean_toilet_robot::callArmEmpty(arm_grab_client, empty_srv, "Grab close")) { pub.publish(geometry_msgs::Twist()); return 1; }
    if (!clean_toilet_robot::waitClaw(pub,2.5,"Grab close")) return 1;

    // 机械臂抬起
    // Keep the same lateral position while lifting the brush out of its holder.
    move_srv.request.x = grab_lateral_compensation_mm;
    move_srv.request.y = 300;
    move_srv.request.z = 190;
    ROS_INFO("Grab lift target: x=%.0f mm, y=%.0f mm, z=%.0f mm (saved lateral compensation)",
             move_srv.request.x, move_srv.request.y, move_srv.request.z);
    if (!clean_toilet_robot::callArmPosition(arm_move_close_client, move_srv, "Grab lift")) { pub.publish(geometry_msgs::Twist()); return 1; }
    sleep(1.0);

    // Return the arm during retreat, after initial clearance.
    std::signal(SIGINT, request_retreat_stop);
    std::signal(SIGTERM, request_retreat_stop);
    const auto retreatActive = []() { return ros::ok() && !retreat_stop_requested; };
    RetreatArmOverlap arm_return(pub, retreat_arm_settings, "Grab", [arm_move_close_client]() mutable {
            upros_message::ArmPosition srv;
            srv.request.x = 0;
            srv.request.y = 300;
            srv.request.z = 210;
            return clean_toilet_robot::callArmPosition(arm_move_close_client, srv, "Grab carry");
        }, retreatActive);
    vel_msg = geometry_msgs::Twist();
    vel_msg.linear.x = -0.15;
    ros::WallRate retreat_rate(retreat_frequency_hz);
    for (int retreat_tick = 0; retreatActive() && retreat_tick < retreat_ticks; ++retreat_tick)
    {
        if (!arm_return.update()) { arm_return.cancel(); return 1; }
        pub.publish(vel_msg);
        ros::spinOnce();
        retreat_rate.sleep();
    }
    pub.publish(geometry_msgs::Twist());
    if (!retreatActive() || !arm_return.finish()) { arm_return.cancel(); return 1; }

    action_timing.complete();
    ros::shutdown();

    return 0;
}
