#ifndef CLEAN_TOILET_ROBOT_ARM_SERVICE_CHECKS_H
#define CLEAN_TOILET_ROBOT_ARM_SERVICE_CHECKS_H
#include <ros/ros.h>
#include <upros_message/ArmPosition.h>
#include <std_srvs/Empty.h>
#include <upros_message/ServoCommand.h>
#include <cmath>
#include <ros/master.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <functional>
#include <string>
#include <algorithm>
#include <geometry_msgs/Twist.h>
namespace clean_toilet_robot {
// 服务状态表示逆解有效且已接受指令；不表示舵机到位，原等待时间继续保留。
inline bool callArmPosition(ros::ServiceClient &client, upros_message::ArmPosition &srv,
                            const char *stage) {
    if (!ros::ok()) return false;
    const auto started = ros::WallTime::now();
    if (!std::isfinite(srv.request.x) || !std::isfinite(srv.request.y) ||
        !std::isfinite(srv.request.z)) {
        ROS_ERROR("Arm %s: non-finite request; command cancelled", stage);
        return false;
    }
    if (!client.call(srv)) {
        ROS_ERROR("Arm %s: service transport failed", stage);
        return false;
    }
    if (srv.response.status != 1) {
        ROS_ERROR("Arm %s: position rejected (status=%d); check IK and updated arm service",
                  stage, static_cast<int>(srv.response.status));
        return false;
    }
    ROS_INFO("Arm %s accepted: x=%.1f, y=%.1f, z=%.1f mm; service=%.3f s; motion completion still waits",
             stage, srv.request.x, srv.request.y, srv.request.z,
             (ros::WallTime::now() - started).toSec());
    return true;
}
inline bool callArmEmpty(ros::ServiceClient &client, std_srvs::Empty &srv,
                         const char *stage) {
    if (!ros::ok()) return false;
    const auto started = ros::WallTime::now();
    if (!client.call(srv)) {
        ROS_ERROR("Arm %s: service transport failed", stage);
        return false;
    }
    ROS_INFO("Arm %s command accepted: service=%.3f s; motion completion still waits",
             stage, (ros::WallTime::now() - started).toSec());
    return true;
}
// 开始动作前检查机械臂服务和驱动均已更新，防止混用旧服务造成虚假成功。
inline bool requireArmTransport(ros::NodeHandle &nh) {
    auto ready=nh.serviceClient<std_srvs::Empty>("/upros_arm_control/serial_ready");
    std_srvs::Empty req;
    if (!ready.waitForExistence(ros::Duration(0.2)) || !ready.call(req)) {
        ROS_ERROR("Updated arm/serial backend not ready; restart robot bringup before running actions"); return false;
    }
    return true;
}
// 单轴动作同样走串口确认，保留原角度、速度和协议。
inline bool callSingleServo(ros::NodeHandle &nh, const upros_message::SingleServo &cmd, const char *stage) {
    if (!ros::ok()) return false;
    auto client=nh.serviceClient<upros_message::ServoCommand>("/zoo_driver/servo_command");
    if (!client.waitForExistence(ros::Duration(0.2))) { ROS_ERROR("%s: servo transport service unavailable",stage); return false; }
    upros_message::ServoCommand srv; srv.request.commands.push_back(cmd);
    srv.request.single_frame=true; srv.request.timeout_sec=0.1;
    if (!client.call(srv) || !srv.response.sent) { ROS_ERROR("%s: serial send unconfirmed: %s",stage,srv.response.message.c_str()); return false; }
    ROS_INFO("%s: id=%u angle=%d speed=%u, serial write=%.4f s; not arrival feedback",stage,cmd.ID,cmd.Target_position_Angle,cmd.Rotation_Speed,srv.response.elapsed_sec);
    return true;
}
// 关键夹爪阶段在停车状态独立等待，使用墙钟，取消后禁止下一动作。
inline bool waitClaw(ros::Publisher &pub,double seconds,const char *stage) {
    const auto started=ros::WallTime::now();
    const auto deadline=started+ros::WallDuration(seconds);
    while (ros::ok() && ros::WallTime::now()<deadline) {
        pub.publish(geometry_msgs::Twist()); ros::spinOnce();
        ros::WallDuration(std::max(0.0,std::min(0.02,(deadline-ros::WallTime::now()).toSec()))).sleep();
    }
    pub.publish(geometry_msgs::Twist());
    ROS_INFO("%s: conservative claw wait %.3f s complete=%d; no physical arrival feedback",stage,(ros::WallTime::now()-started).toSec(),ros::ok());
    return ros::ok();
}
// 同一 ROS master 的动作互斥；进程退出自动释放，防止抓刷、放刷或其他动作互相覆盖。
// 独立测试 master 使用不同锁文件，不会干扰实机节点。
class ArmActionLock {
public:
    ArmActionLock() {
        const std::string path="/tmp/clean_toilet_arm_"+std::to_string(std::hash<std::string>{}(ros::master::getURI()))+".lock";
        fd_=::open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
        locked_=fd_>=0 && ::flock(fd_,LOCK_EX|LOCK_NB)==0;
        if (!locked_) ROS_ERROR("Another cleaning arm action is running or action lock unavailable; no motion permitted");
    }
    ~ArmActionLock() { if(fd_>=0) ::close(fd_); }
    bool ready() const { return locked_; }
    ArmActionLock(const ArmActionLock&)=delete;
    ArmActionLock& operator=(const ArmActionLock&)=delete;
private:
    int fd_=-1; bool locked_=false;
};
class ActionTiming {
public:
    explicit ActionTiming(const char *name) : name_(name), started_(ros::WallTime::now()) {
        ROS_INFO("Action %s started; CPP defaults are used unless explicitly overridden", name_);
    }
    void complete() { completed_ = true; report(); }
    ~ActionTiming() { report(); }
private:
    void report() {
        if (reported_) return;
        reported_ = true;
        ROS_INFO("Action %s finished: elapsed=%.3f s, complete=%d", name_,
                 (ros::WallTime::now() - started_).toSec(), completed_);
    }
    const char *name_;
    ros::WallTime started_;
    bool completed_ = false;
    bool reported_ = false;
};
}
#endif
