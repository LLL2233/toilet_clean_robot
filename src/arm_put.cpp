#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "upros_message/ArmPosition.h"
#include "upros_message/SingleServo.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "clean_toilet_robot/arm_service_checks.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "clean_toilet_robot/tag_recovery.h"
#include "clean_toilet_robot/tag_alignment.h"
using clean_toilet_robot::TagAlignment;
#include "clean_toilet_robot/base_stop_confirmation.h"
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
const double max_put_lateral_compensation_mm = 20.0;

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
    clean_toilet_robot::ActionTiming action_timing("Put");
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

    const int retreat_ticks = 21;
    const double retreat_frequency_hz = 10.0;
    const double retreat_seconds = retreat_ticks / retreat_frequency_hz;
    RetreatArmOverlap::Settings retreat_arm_settings;
    // 放刷后先后退 1 秒，再异步回零，与剩余后退重叠。
    retreat_arm_settings.start_delay = 1.0;
    if (!RetreatArmOverlap::readSettings(private_nh_, retreat_seconds, 0.5, retreat_arm_settings)) return 1;

    // 使用到的 service
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_hor_service_close");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    ros::Publisher single_joint_pub = nh.advertise<upros_message::SingleServo>("/single_servo_topic", 10);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber tag_sub_ = nh.subscribe("tag_detections", 1, tagCallback);
    TagRecovery recovery(nh, private_nh_, pub);

    // 放刷前进后：至少缓冲 0.3 秒，同时连续停稳 0.2 秒；最长等 1.7 秒。
    // 缓冲用于给刷子留出稳定时间，底盘里程计无法直接测量刷子的晃动。
    const double put_stop_min_buffer_sec = 0.30;
    const double put_stop_stable_sec = 0.20;
    const double put_stop_timeout_sec = 1.70;
    clean_toilet_robot::BaseStopConfirmation stop_confirmation(nh, pub, alignment_odom_topic,
        put_stop_min_buffer_sec, put_stop_stable_sec, put_stop_timeout_sec);


    ros::WallRate loop_rate(10); // Preserve task approach/retreat timing.
    ros::WallRate alignment_rate(50);
    TagAlignment alignment(nh, pub, min_angular_speed, min_linear_speed, fine_tolerance,
                           coarse_threshold, pulse_seconds, settle_seconds, startup_timeout, max_pulses,
                           alignment_odom_topic, z_target_distance, target_z_tolerance, target_z_tolerance);

    geometry_msgs::Twist vel_msg;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    double put_lateral_compensation_mm = 0.0;

   // 首先，对齐到下方的ApriTag码
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
                ROS_ERROR("Cannot save put compensation without a tag coordinate frame; putting cancelled");
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
                ROS_ERROR("Cannot transform saved put horizontal error; putting cancelled: %s", ex.what());
                return 1;
            }
            if (!std::isfinite(arm_offset.vector.y))
            {
                ROS_ERROR("Non-finite put compensation; putting cancelled");
                return 1;
            }
            // ArmPosition uses the same x=-arm_base_link.y mapping as arm_put,
            // with millimetres; the arm solver ultimately uses integer mm.
            // AprilTag x describes the TAG relative to the camera, not the
            // robot's displacement: robot-right means tag-left (negative x).
            // Move the endpoint toward the tag, opposite the robot's bias.
            const double requested_offset_mm = -arm_offset.vector.y * 1000.0;
            put_lateral_compensation_mm = std::round(std::max(-max_put_lateral_compensation_mm,
                std::min(max_put_lateral_compensation_mm, requested_offset_mm)));
            ROS_INFO("Saved put horizontal residual: camera_x=%+.1f mm (%u frames), arm_x_compensation=%+.0f mm",
                     saved_horizontal_error * 1000.0, confirmed.frames, put_lateral_compensation_mm);
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

    // 第二步，运动到放置位置
    upros_message::ArmPosition move_srv;
    // 按当前 CPP 保留残差倍数；本次不改变末端补偿标定。
    move_srv.request.x = 1.5*put_lateral_compensation_mm;
    move_srv.request.y = 300;
    move_srv.request.z = 200;
    ROS_INFO("Put positioning target: tag_x=%d mm, saved_compensation=%+.0f mm, final_x=%.0f mm, y=%.0f mm, z=%.0f mm",
             x, put_lateral_compensation_mm, move_srv.request.x, move_srv.request.y, move_srv.request.z);
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
    if (!clean_toilet_robot::callArmPosition(arm_move_close_client, move_srv, "Put position/lower")) { pub.publish(geometry_msgs::Twist()); return 1; }
    sleep(1.0);

    // 夹爪先松一下，让马桶刷垂直
    upros_message::SingleServo single_servo;
    single_servo.ID = 6;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = -400;
    if (!clean_toilet_robot::callSingleServo(nh,single_servo,"Put loosen")) { pub.publish(geometry_msgs::Twist()); return 1; }
    if (!clean_toilet_robot::waitClaw(pub,1.5,"Put loosen")) return 1;

    // 夹爪再紧一下
    single_servo.ID = 6;
    single_servo.Rotation_Speed = 50;
    single_servo.Target_position_Angle = -600;
    if (!clean_toilet_robot::callSingleServo(nh,single_servo,"Put tighten")) { pub.publish(geometry_msgs::Twist()); return 1; }
    if (!clean_toilet_robot::waitClaw(pub,1.0,"Put tighten")) return 1;

    // 先前进一小段,大约15cm
    vel_msg.linear.x = 0.2;
    int count = 0;
    while (ros::ok() && count < 9)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下：最短缓冲与停稳确认并行，确认成功才允许向下放刷。
    vel_msg = geometry_msgs::Twist();
    pub.publish(vel_msg);
    if (!stop_confirmation.wait([]() { return ros::ok(); }, "put_before_lower")) return 1;

    // 向下运动，放入容器区域
    move_srv.request.x = 2.5*put_lateral_compensation_mm;
    move_srv.request.y = 300.0;
    move_srv.request.z = 120.0;
    ROS_INFO("Put lowering target: x=%.0f mm, y=%.0f mm, z=%.0f mm (saved lateral compensation)",
             move_srv.request.x, move_srv.request.y, move_srv.request.z);
    if (!clean_toilet_robot::callArmPosition(arm_move_close_client, move_srv, "Put position/lower")) { pub.publish(geometry_msgs::Twist()); return 1; }
    sleep(1.0);

    // 第三步，打开夹爪，放手
    std_srvs::Empty empty_srv;
    if (!clean_toilet_robot::callArmEmpty(arm_release_client, empty_srv, "Put release")) { pub.publish(geometry_msgs::Twist()); return 1; }
    // 释放到回零的间隔约 1.5 秒，不独立等待 2.5 秒。
    // CPP 默认后退 1.0 秒启动回零，所以释放后仅停车等 0.5 秒，再边后退边回零。
    // 显式延长后退回零延迟时，保留更长的离开容器时间。
    const double put_release_to_home_interval_sec = 1.5;
    const double release_wait_sec = std::max(0.0, put_release_to_home_interval_sec - retreat_arm_settings.start_delay);
    ROS_INFO("Put release/home timing: stationary_wait=%.2f s, retreat_before_home=%.2f s, total_interval=%.2f s",
             release_wait_sec, retreat_arm_settings.start_delay, release_wait_sec + retreat_arm_settings.start_delay);
    if (!clean_toilet_robot::waitClaw(pub,release_wait_sec,"Put release")) return 1;
    
    // Return the arm during retreat, after initial clearance.
    std::signal(SIGINT, request_retreat_stop);
    std::signal(SIGTERM, request_retreat_stop);
    const auto retreatActive = []() { return ros::ok() && !retreat_stop_requested; };
    RetreatArmOverlap arm_return(pub, retreat_arm_settings, "Put", [arm_zero_client]() mutable {
            std_srvs::Empty srv;
            return clean_toilet_robot::callArmEmpty(arm_zero_client, srv, "Put home");
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
