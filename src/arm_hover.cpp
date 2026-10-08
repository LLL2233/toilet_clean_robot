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
#include "clean_toilet_robot/hover_base_sweep.h"
using clean_toilet_robot::TagAlignment;
#include "upros_message/SingleServo.h"
#include <cmath>
#include <string>
#include <geometry_msgs/Vector3Stamped.h>
#include <algorithm>
#include <csignal>
#include <future>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
    stop_requested = 1;
}

bool taskActive()
{
    return ros::ok() && !stop_requested;
}

bool waitSeconds(double second)
{
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(second);
    while (taskActive() && ros::WallTime::now() < deadline)
        ros::WallDuration(0.05).sleep();
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
const double max_alignment_linear_speed = 0.10;
const double max_alignment_angular_speed = 0.50;

// PID控制参数
const double Kp = 3;                    // 比例系数
const double target_x_tolerance = 0.02; // X轴位置容忍误差

const double z_target_distance = 0.15;  // 到二维码的距离
const double target_z_tolerance = 0.02; // 距离容忍度误差
const double max_hover_lateral_compensation_mm = 20.0;

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
        if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) || position.z <= 0.0)
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
    clean_toilet_robot::ActionTiming action_timing("Hover");
    ros::NodeHandle transport_nh;
    if (!clean_toilet_robot::requireArmTransport(transport_nh)) return 1;
    ros::NodeHandle private_nh("~");
    private_nh.param("tag", tag_id, 1);

    // 两种候选方案共用以下参数，切换时不需要改动这里。
    // 请在“第三步”的两段相邻代码块中，注释一段、取消另一段的注释。
    // 底盘扫动：名义左转 15 度、右转 30 度，最后大致回中。
    double sweep_angle_deg = 15.0;
    double sweep_speed = 0.60;
    double sweep_settle_sec = 0.20;
    double sweep_timeout_sec = 4.0;
    double sweep_brake_lead_deg = 8.0; // 0.6 rad/s 实测停车后仍转约 7.6 度，提前约 8 度刹停。
    private_nh.param("hover_sweep_angle_deg", sweep_angle_deg, sweep_angle_deg);
    private_nh.param("hover_sweep_angular_speed", sweep_speed, sweep_speed);
    private_nh.param("hover_sweep_settle_sec", sweep_settle_sec, sweep_settle_sec);
    private_nh.param("hover_sweep_turn_timeout_sec", sweep_timeout_sec, sweep_timeout_sec);
    private_nh.param("hover_sweep_brake_lead_deg", sweep_brake_lead_deg, sweep_brake_lead_deg);
    if (!std::isfinite(sweep_angle_deg) || sweep_angle_deg < 1.0 || sweep_angle_deg > 20.0 ||
        !std::isfinite(sweep_speed) || sweep_speed < 0.30 || sweep_speed > 0.60 ||
        !std::isfinite(sweep_settle_sec) || sweep_settle_sec < 0.20 || sweep_settle_sec > 1.0 ||
        !std::isfinite(sweep_timeout_sec) || sweep_timeout_sec < 2.0 || sweep_timeout_sec > 8.0 ||
        !std::isfinite(sweep_brake_lead_deg) || sweep_brake_lead_deg < 0.0 || sweep_brake_lead_deg > 12.0)
    {
        ROS_ERROR("Invalid hover sweep settings: angle [1,20] deg, speed [0.30,0.60] rad/s, settle [0.20,1.0] s, turn timeout [2,8] s, brake lead [0,12] deg");
        return 1;
    }
    // 原方案参数：保留定义，方便直接取消自旋动作代码的注释。
    // 舵机速度默认 100，旋转和回转各等待 2 秒，完成效果以实机为准。
    int servo5_speed = 100;
    double servo5_turn_wait = 2.0;
    double servo5_return_wait = 2.0;
    private_nh.param("servo5_speed", servo5_speed, servo5_speed);
    private_nh.param("servo5_turn_wait", servo5_turn_wait, servo5_turn_wait);
    private_nh.param("servo5_return_wait", servo5_return_wait, servo5_return_wait);
    // 舵机速度的软件上限为 150，与现有机械臂动作保持一致。
    if (servo5_speed < 1 || servo5_speed > 150 ||
        !std::isfinite(servo5_turn_wait) || servo5_turn_wait < 0.5 || servo5_turn_wait > 10.0 ||
        !std::isfinite(servo5_return_wait) || servo5_return_wait < 0.5 || servo5_return_wait > 10.0)
    {
        ROS_ERROR("Invalid hover servo5 settings: speed must be within [1, 150], both waits within [0.5, 10.0] seconds");
        return 1;
    }
    ROS_INFO("Hover servo5 settings: speed=%d, turn_wait=%.2f s, return_wait=%.2f s",
             servo5_speed, servo5_turn_wait, servo5_return_wait);

    // 后退离开作业位置一段时间后，开始并行复位机械臂。
    double home_start_delay = 0.5;
    double home_wait_seconds = 1.0;
    private_nh.param("hover_home_start_delay_sec", home_start_delay, home_start_delay);
    private_nh.param("hover_home_wait_sec", home_wait_seconds, home_wait_seconds);
    if (!std::isfinite(home_start_delay) || home_start_delay < 0.1 || home_start_delay > 2.5 ||
        !std::isfinite(home_wait_seconds) || home_wait_seconds < 0.5 || home_wait_seconds > 5.0)
    {
        ROS_ERROR("Invalid hover home settings: start delay within [0.1, 2.5] s, wait within [0.5, 5.0] s");
        return 1;
    }
    ROS_INFO("Hover retreat/home overlap: start_delay=%.2f s, home_wait=%.2f s",
             home_start_delay, home_wait_seconds);

    // 实机测试：底盘约从 0.27 rad/s 开始转动，对准默认使用 0.30 rad/s。
    double min_angular_speed = 0.30;
    double min_linear_speed = 0.04;
    private_nh.param("alignment_min_angular_speed", min_angular_speed, min_angular_speed);
    private_nh.param("alignment_min_linear_speed", min_linear_speed, min_linear_speed);
    if (!std::isfinite(min_angular_speed) || min_angular_speed < 0.28 || min_angular_speed > max_alignment_angular_speed ||
        !std::isfinite(min_linear_speed) || min_linear_speed < 0.0 || min_linear_speed > max_alignment_linear_speed)
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
    private_nh.param("alignment_fine_tolerance_m", fine_tolerance, fine_tolerance);
    private_nh.param("alignment_coarse_threshold_m", coarse_threshold, coarse_threshold);
    private_nh.param("alignment_pulse_sec", pulse_seconds, pulse_seconds);
    private_nh.param("alignment_settle_sec", settle_seconds, settle_seconds);
    private_nh.param("alignment_startup_timeout_sec", startup_timeout, startup_timeout);
    private_nh.param("alignment_max_pulses", max_pulses, max_pulses);
    private_nh.param("recovery_odom_topic", alignment_odom_topic, alignment_odom_topic);
    if (!std::isfinite(fine_tolerance) || fine_tolerance < 0.005 || fine_tolerance > 0.02 ||
        !std::isfinite(coarse_threshold) || coarse_threshold <= 0.02 || coarse_threshold > 0.15 ||
        !std::isfinite(pulse_seconds) || pulse_seconds < 0.06 || pulse_seconds > 0.25 ||
        !std::isfinite(settle_seconds) || settle_seconds < 0.10 || settle_seconds > 0.80 ||
        !std::isfinite(startup_timeout) || startup_timeout < 0.15 || startup_timeout > 1.0 ||
        max_pulses < 1 || max_pulses > 12 || alignment_odom_topic.empty())
    {
        ROS_ERROR("Invalid hover pulse alignment parameters; task not started");
        return 1;
    }
    ROS_INFO("Hover pulse alignment: fine=%.0f mm, coarse=%.0f mm, pulse=%.3f s, settle=%.3f s, startup_timeout=%.3f s, max_pulses=%d",
             fine_tolerance * 1000.0, coarse_threshold * 1000.0,
             pulse_seconds, settle_seconds, startup_timeout, max_pulses);

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_hor_service_open");
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_close");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 保留舵机发布器，取消自旋代码注释后即可使用；建立发布器本身不会发出旋转指令。
    ros::Publisher single_joint_pub = nh.advertise<upros_message::SingleServo>("/single_servo_topic", 10);

    ros::Subscriber tag_sub_ = nh.subscribe("tag_detections", 1, tagCallback);
    TagRecovery recovery(nh, private_nh, pub);

    ros::WallRate loop_rate(10); // 保持原有前进和后退的时间与距离。
    ros::WallRate alignment_rate(50);
    TagAlignment alignment(nh, pub, min_angular_speed, min_linear_speed, fine_tolerance,
                             coarse_threshold, pulse_seconds, settle_seconds, startup_timeout, max_pulses,
                             alignment_odom_topic, z_target_distance, target_z_tolerance, target_z_tolerance);

    clean_toilet_robot::HoverBaseSweep base_sweep(nh, pub, alignment_odom_topic,
        sweep_angle_deg, sweep_speed, sweep_settle_sec, sweep_timeout_sec, sweep_brake_lead_deg);

    geometry_msgs::Twist vel_msg;
    const auto stop = [&pub]() {
        pub.publish(geometry_msgs::Twist());
        ros::WallDuration(0.1).sleep();
    };
    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener(transform_buffer);
    double hover_lateral_compensation_mm = 0.0;
    const ros::WallTime alignment_started = ros::WallTime::now();
    while (taskActive() && !aim_to_tag)
    {
        ros::spinOnce();
        if (!taskActive()) break;
        geometry_msgs::Twist cmd;
        const bool tag_fresh = targetIsFresh();
        const bool within_tolerance =
            std::fabs(current_x) <= alignment.tolerance() &&
            std::fabs(current_z - z_target_distance) <= target_z_tolerance;
        if ((ros::WallTime::now() - alignment_started).toSec() >= alignment_timeout)
        {
            stop();
            ROS_ERROR("AprilTag alignment timed out after %.0f seconds: fresh=%d, x_error=%.1f mm, z_error=%.1f mm",
                      alignment_timeout, tag_fresh, current_x * 1000.0,
                      (current_z - z_target_distance) * 1000.0);
            return 1;
        }
        const TagRecovery::Result recovery_result =
            recovery.update(tag_fresh, valid_tag_frames, within_tolerance, true);
        if (recovery_result == TagRecovery::Result::Failed)
        {
            stop();
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
            stop();
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
            stop();
            alignment.resetMotion();
            alignment_rate.sleep();
            continue;
        }
        if (std::fabs(current_x) <= alignment.tolerance() &&
            std::fabs(current_z - z_target_distance) <= target_z_tolerance)
        {
            pub.publish(cmd);
            // 这一组已由共享对准模块确认，不再等待另外三帧。
            const double saved_horizontal_error = confirmed.mean_x;
            if (confirmed.coordinate_frame.empty())
            {
                ROS_ERROR("Cannot save hover compensation without a tag coordinate frame; hovering cancelled");
                stop();
                return 1;
            }
            // 只转换误差位移向量，避免把相机安装平移量加入对准残差。
            geometry_msgs::Vector3Stamped camera_offset;
            camera_offset.header.frame_id = confirmed.coordinate_frame;
            camera_offset.header.stamp = confirmed.last_pose_stamp.isZero() ? confirmed.last_frame_stamp : confirmed.last_pose_stamp;
            camera_offset.vector.x = saved_horizontal_error;
            geometry_msgs::Vector3Stamped arm_offset;
            try
            {
                arm_offset = transform_buffer.transform(camera_offset, "arm_base_link", ros::Duration(0.5));
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_ERROR("Cannot transform saved hover horizontal error; hovering cancelled: %s", ex.what());
                stop();
                return 1;
            }
            if (!std::isfinite(arm_offset.vector.y))
            {
                ROS_ERROR("Non-finite hover compensation; hovering cancelled");
                stop();
                return 1;
            }
            // 与 arm_put 一样，机械臂请求 x 对应 -arm_base_link.y，单位为毫米。
            // 底层解算最终使用整数毫米。标签 x 是标签相对相机的位置：
            // 机器人偏右时，标签在相机左侧，x 为负。
            // 让末端朝标签方向调整，以补偿机器人的偏移。
            const double requested_offset_mm = -arm_offset.vector.y * 1000.0;
            hover_lateral_compensation_mm = std::round(std::max(-max_hover_lateral_compensation_mm,
                std::min(max_hover_lateral_compensation_mm, requested_offset_mm)));
            ROS_INFO("Saved hover horizontal residual: camera_x=%+.1f mm (%u frames), arm_x_compensation=%+.0f mm",
                     saved_horizontal_error * 1000.0, confirmed.frames, hover_lateral_compensation_mm);
            ROS_INFO("AprilTag alignment complete after 3 distinct aligned frames: x_error=%.1f mm, z_error=%.1f mm",
                     confirmed.mean_x * 1000.0, (confirmed.mean_z - z_target_distance) * 1000.0);
            aim_to_tag = true;
            break;
        }
        alignment_rate.sleep();
    }
    stop();
    if (!taskActive() || !aim_to_tag) return 1;

    // 第一步，运动到感应
    upros_message::ArmPosition move_srv;
    move_srv.request.x = 0.5*hover_lateral_compensation_mm;
    move_srv.request.y = 330.0;
    move_srv.request.z = 120.0;
    ROS_INFO("Hover sensing target: x=%.0f mm, y=%.0f mm, z=%.0f mm (saved lateral compensation)",
             move_srv.request.x, move_srv.request.y, move_srv.request.z);
    ros::spinOnce();
    TagAlignment::ConfirmedObservation before_arm;
    std::string shared_confirmation_reason;
    const bool shared_confirmation_valid = alignment.getConfirmedObservation(before_arm, &shared_confirmation_reason);
    if (!targetIsFresh() || !shared_confirmation_valid ||
        current_tag_frame != before_arm.coordinate_frame ||
        std::fabs(current_x) > alignment.tolerance() || !alignment.depthWithin(current_z))
    {
        stop();
        ROS_ERROR("AprilTag confirmation blocked before task motion: reason=%s, tag_fresh=%d, image_age=%.3f s, current_x=%+.1f mm, current_z=%.1f mm, observed_frame=%s, confirmed_frame=%s; task stopped",
                  shared_confirmation_reason.empty() ? "current tag frame or range changed" : shared_confirmation_reason.c_str(),
                  targetIsFresh(), (ros::Time::now() - last_frame_stamp).toSec(),
                  current_x * 1000.0, current_z * 1000.0,
                  current_tag_frame.c_str(), before_arm.coordinate_frame.c_str());
        return 1;
    }
    if (!clean_toilet_robot::callArmPosition(arm_move_close_client, move_srv, "Hover position"))
    {
        stop();
        ROS_ERROR("Arm positioning service failed; cleaning aborted.");
        return 1;
    }
    if (!taskActive() || !waitSeconds(1.0))
    {
        stop();
        return 1;
    }

    // 第二步，前进一段距离
    vel_msg = geometry_msgs::Twist();
    vel_msg.linear.x = 0.1;
    int count = 0;
    while (taskActive() && count < 15)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    ROS_INFO("Hover approach complete: stop requested; fresh odometry must confirm linear/angular settling before sweep");
    stop();
    if (!taskActive()) return 1;

    // 第三步：两种候选方案，只启用其中一段，完成后统一进入后退和复位。
    // 切回 5 号舵机方案：注释【方案一】整个 if 块，删除【方案二】外层的 /* 和 */。
    // 切回底盘方案：恢复【方案一】if 块，并把【方案二】整段放入 /* 和 */ 之间。

    // ===== 方案一：底盘左右扫动（当前启用） =====
    // 不再固定等待 0.5 秒；扫动模块检查新鲜里程计，连续停稳 0.2 秒后开始。
    // 默认左转约 15 度、右转约 30 度，最后大致回中，不做精确角度修正。
    if (!base_sweep.run(taskActive))
    {
        stop();
        return 1;
    }
    // ===== 方案一结束 =====

    // ===== 方案二：原来的 5 号舵机自旋（当前整段注释） =====
/*
    // 切回舵机方案时，保留原来的前进后 0.5 秒缓冲。
    if (!taskActive() || !waitSeconds(0.5)) return 1;
    // 5 号舵机转到 -148.0 度，增强触发；角度字段以 0.1 度为单位。
    upros_message::SingleServo single_servo;
    single_servo.ID = 5;
    single_servo.Rotation_Speed = servo5_speed;
    single_servo.Target_position_Angle = -1480;
    if (!clean_toilet_robot::callSingleServo(nh,single_servo,"Hover servo5")) { stop(); return 1; }
    if (!waitSeconds(servo5_turn_wait))
    {
        stop();
        return 1;
    }
    // 5 号舵机转回原位置。
    single_servo.Target_position_Angle = 0;
    if (!clean_toilet_robot::callSingleServo(nh,single_servo,"Hover servo5")) { stop(); return 1; }
    if (!waitSeconds(servo5_return_wait))
    {
        stop();
        return 1;
    }
*/
    // ===== 方案二结束 =====

    // 第四步：按当前 CPP 的 15 个 100 毫秒周期后退，同时复位机械臂。
    // 复位服务独立调用，避免等待响应时中断底盘速度更新。
    vel_msg = geometry_msgs::Twist();
    vel_msg.linear.x = -0.2;
    count = 0;
    const ros::WallTime retreat_started = ros::WallTime::now();
    ros::WallRate retreat_rate(10);
    ros::WallTime home_requested_at, home_ready_deadline;
    std::future<bool> home_call;
    bool home_requested = false;
    bool home_accepted = false;
    const double home_service_timeout = 2.0;
    const auto pollHome = [&]() -> bool {
        if (!home_requested || home_accepted) return true;
        if (home_call.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
            if (!home_call.get())
            {
                ROS_ERROR("Hover arm zero service failed; retreat stopped");
                return false;
            }
            home_accepted = true;
            // 从服务响应成功时开始计时，留出复位动作等待时间。
            home_ready_deadline = ros::WallTime::now() + ros::WallDuration(home_wait_seconds);
            ROS_INFO("Hover home command accepted during retreat; %.2f s settling overlaps remaining retreat",
                     home_wait_seconds);
        }
        else if ((ros::WallTime::now() - home_requested_at).toSec() >= home_service_timeout)
        {
            ROS_ERROR("Hover arm zero response timed out; retreat stopped");
            return false;
        }
        return true;
    };
    const auto cancelHome = [&]() {
        stop();
        // 关闭 ROS 连接，解除尚未返回的服务调用，再等待线程结束。
        ros::shutdown();
        if (home_call.valid()) home_call.wait();
    };
    while (taskActive() && count < 15)
    {
        const double retreat_elapsed = (ros::WallTime::now() - retreat_started).toSec();
        if (!home_requested && retreat_elapsed >= home_start_delay)
        {
            home_requested = true;
            home_requested_at = ros::WallTime::now();
            ROS_INFO("Hover starting arm home while retreating: elapsed=%.2f s, planned_clearance=%.0f mm",
                     retreat_elapsed, retreat_elapsed * std::fabs(vel_msg.linear.x) * 1000.0);
            home_call = std::async(std::launch::async, [arm_zero_client]() mutable {
                std_srvs::Empty empty_srv;
                return clean_toilet_robot::callArmEmpty(arm_zero_client, empty_srv, "Hover home");
            });
        }
        if (!pollHome()) { cancelHome(); return 1; }
        pub.publish(vel_msg);
        ros::spinOnce();
        retreat_rate.sleep();
        count++;
    }
    stop();
    if (!taskActive() || !home_requested) { cancelHome(); return 1; }

    // 仅补足后退过程中尚未覆盖的复位等待时间，持续发布停车指令。
    while (taskActive())
    {
        if (!pollHome()) { cancelHome(); return 1; }
        if (home_accepted && ros::WallTime::now() >= home_ready_deadline) break;
        pub.publish(geometry_msgs::Twist());
        ros::spinOnce();
        ros::WallDuration(0.05).sleep();
    }
    if (!taskActive()) { cancelHome(); return 1; }
    ROS_INFO("Hover retreat and home settling complete");
    stop();
    action_timing.complete();
    ros::shutdown();

    return 0;
}
