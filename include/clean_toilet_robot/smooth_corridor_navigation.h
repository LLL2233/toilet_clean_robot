#ifndef CLEAN_TOILET_ROBOT_SMOOTH_CORRIDOR_NAVIGATION_H
#define CLEAN_TOILET_ROBOT_SMOOTH_CORRIDOR_NAVIGATION_H

#include <ros/ros.h>
#include <toilet_route_planner/RouteControl.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

namespace clean_toilet_robot {

// 只在 true 分支的起始通道使用；正常到点、失败退出时都恢复原前进速度。
// 不修改位置容忍、角度容忍、膨胀半径、碰撞检测或最终停车条件。
class CorridorCornerSpeed {
public:
    explicit CorridorCornerSpeed(ros::NodeHandle& nh)
        : nh_(nh), service_(nh.serviceClient<dynamic_reconfigure::Reconfigure>(
              "/move_base/TebLocalPlannerROS/set_parameters")) {}
    ~CorridorCornerSpeed() { if (changed_) restore(); }
    bool initialize(double corner_speed, double slow_radius) {
        std::string planner;
        if (!nh_.getParam("/move_base/base_local_planner",planner) || planner!="teb_local_planner/TebLocalPlannerROS" ||
            !nh_.getParam("/move_base/TebLocalPlannerROS/max_vel_x",original_) ||
            !nh_.getParam("/move_base/TebLocalPlannerROS/acc_lim_x",acceleration_) ||
            !std::isfinite(original_) || original_<0.04 || !std::isfinite(acceleration_) || acceleration_<=0.0 ||
            !service_.waitForExistence(ros::Duration(2.0))) {
            ROS_ERROR("Smooth corridor: cannot read/configure TEB corner speed"); return false;
        }
        current_=original_; corner_=std::min(corner_speed,original_); radius_=slow_radius;
        ROS_INFO("Smooth corridor speeds: straight_max=%.3f m/s, corner_max=%.3f m/s, slow_radius=%.2f m",
            original_,corner_,radius_); return true;
    }
    bool update(double nearest_guide_distance) {
        if ((ros::WallTime::now()-last_update_).toSec()<0.20) return true;
        const double limit=std::min(original_,std::sqrt(corner_*corner_+2.0*acceleration_*
            std::max(0.0,nearest_guide_distance-radius_)));
        if (std::fabs(limit-current_)<0.025) return true;
        changed_=true; last_update_=ros::WallTime::now();
        if (!set(limit)) return false;
        current_=limit;
        ROS_INFO_THROTTLE(1.0,"Smooth corridor corner speed limit: %.3f m/s",limit); return true;
    }
    bool restore() {
        if (!changed_) return true;
        for (int attempt=0; attempt<3; ++attempt) {
            if (set(original_)) {changed_=false;current_=original_;return true;}
            ros::WallDuration(0.10).sleep();
        }
        ROS_ERROR("Smooth corridor: could not restore max_vel_x; task stopped, reload navigation before retrying");
        return false;
    }
private:
    bool set(double value) {
        dynamic_reconfigure::Reconfigure request; dynamic_reconfigure::DoubleParameter parameter;
        parameter.name="max_vel_x";parameter.value=value;request.request.config.doubles.push_back(parameter);
        if (!service_.call(request)) return false;
        for (const auto& p : request.response.config.doubles)
            if (p.name=="max_vel_x" && std::isfinite(p.value) && std::fabs(p.value-value)<1e-6) return true;
        return false;
    }
    ros::NodeHandle& nh_; ros::ServiceClient service_;
    double original_=0.0,current_=0.0,acceleration_=0.0,corner_=0.0,radius_=0.0;
    bool changed_=false;ros::WallTime last_update_;
};

// 只发送马桶前的最终目标；中间点是全局路径的引导点，不触发到点停车。
// 正常通过时不发布零速度，不切换 action 目标；遇到障碍或定位异常仍允许停车。
template <typename Client, typename FinalYaw>
bool navigateSmoothCorridor(ros::NodeHandle& nh, ros::Publisher& pub, Client& ac, tf2_ros::Buffer& tf_buffer,
    const std::vector<geometry_msgs::Point>& via, double target_x, double target_y,
    double pass_radius, FinalYaw final_yaw) {
    if (via.empty() || !std::isfinite(target_x) || !std::isfinite(target_y) ||
        !std::isfinite(pass_radius) || pass_radius<0.05 || pass_radius>0.25) return false;
    for (const auto& p : via) if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
    ros::NodeHandle pnh("~"); double timeout=120.0;
    pnh.param("navigation_timeout",timeout,120.0);
    // 拐角附近前进速度上限 0.20 m/s；距离引导点 0.40 m 内使用此上限。
    // 进入减速区前按原 acc_lim_x 连续降低上限，不直接给底盘发送固定速度。
    const double corner_speed=0.20, slow_radius=0.40;
    if (!std::isfinite(timeout) || timeout<1.0 || timeout>600.0) return false;
    auto route=nh.serviceClient<toilet_route_planner::RouteControl>("/move_base/ToiletRoutePlanner/route_control");
    if (!route.waitForExistence(ros::Duration(2.0))) {
        ROS_ERROR("Smooth corridor: route planner unavailable; reload navigation");pub.publish(geometry_msgs::Twist());return false;
    }
    move_base_msgs::MoveBaseGoal goal; goal.target_pose.header.frame_id="map";goal.target_pose.header.stamp=ros::Time::now();
    goal.target_pose.pose.position.x=target_x;goal.target_pose.pose.position.y=target_y;
    const double yaw=std::atan2(target_y-via.back().y,target_x-via.back().x);
    tf2::Quaternion q;q.setRPY(0,0,yaw);goal.target_pose.pose.orientation=tf2::toMsg(q);
    const std::string id="smooth_corridor_"+std::to_string(ros::WallTime::now().toNSec());
    auto clear_route=[&]() {
        toilet_route_planner::RouteControl r;r.request.command=r.request.CLEAR;r.request.route_id=id;
        return route.call(r) && r.response.ok && !r.response.active;
    };
    CorridorCornerSpeed speed(nh);
    if (!speed.initialize(corner_speed,slow_radius)) {pub.publish(geometry_msgs::Twist());return false;}
    toilet_route_planner::RouteControl configure;
    configure.request.command=configure.request.CONFIGURE;configure.request.route_id=id;
    configure.request.target=goal.target_pose;configure.request.via_points=via;configure.request.pass_radius=pass_radius;
    if (!route.call(configure) || !configure.response.ok || !configure.response.active || configure.response.failed ||
        configure.response.passed!=0 || configure.response.total!=via.size() ||
        configure.response.message.find("Smooth corridor v2 armed")!=0) {
        clear_route();pub.publish(geometry_msgs::Twist());
        ROS_ERROR("Smooth corridor: planner is unavailable or still an old version; restart navigation before true mode");
        return false;
    }
    auto stop_failed=[&](const std::string& reason) {
        ac.cancelGoal();ac.waitForResult(ros::Duration(2.0));
        for (int i=0;i<3 && ros::ok();++i) {pub.publish(geometry_msgs::Twist());ros::WallDuration(0.10).sleep();}
        if (!clear_route()) ROS_ERROR("Smooth corridor: route cleanup failed; no following action permitted");
        speed.restore();ROS_ERROR("Smooth corridor stopped: %s",reason.c_str());return false;
    };
    ac.sendGoal(goal);
    ROS_INFO("Smooth corridor: one final goal=(%.6f, %.6f), %zu guide points, release_radius=%.3f m; no intermediate stop goals",
        target_x,target_y,via.size(),pass_radius);
    const ros::WallTime deadline=ros::WallTime::now()+ros::WallDuration(timeout);
    std::size_t released=0;ros::WallTime invalid_since;
    while (ros::ok() && ros::WallTime::now()<deadline) {
        toilet_route_planner::RouteControl status;status.request.command=status.request.STATUS;status.request.route_id=id;
        if (!route.call(status) || !status.response.ok || !status.response.active || status.response.failed ||
            status.response.total!=via.size() || status.response.passed<released || status.response.passed>via.size())
            return stop_failed("Route guiding unavailable: "+status.response.message);
        released=status.response.passed;
        const auto state=ac.getState();
        if (state.isDone()) {
            if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
                return stop_failed("Navigation "+state.toString()+": "+state.getText());
            // 最终目标成功即可，不再把漏过圆形范围当作整段导航失败。
            if (!clear_route()) return stop_failed("Cannot clear guide route");
            if (!speed.restore()) return stop_failed("Cannot restore normal navigation speed");
            pub.publish(geometry_msgs::Twist());ros::WallDuration(0.10).sleep();
            if (!final_yaw()) return stop_failed("Final work heading was not reached");
            ROS_INFO("Smooth corridor complete: final navigation succeeded, guides released=%zu/%zu",released,via.size());return true;
        }
        bool pose_ok=false;double nearest=1e9;
        try {
            const auto tf=tf_buffer.lookupTransform("map","base_link",ros::Time(0),ros::Duration(0.05));
            const double age=(ros::Time::now()-tf.header.stamp).toSec();const auto& t=tf.transform.translation;
            pose_ok=!tf.header.stamp.isZero() && std::isfinite(age) && age>=-0.5 && age<=0.5 &&
                std::isfinite(t.x) && std::isfinite(t.y);
            if (pose_ok) for (const auto& p : via) nearest=std::min(nearest,std::hypot(t.x-p.x,t.y-p.y));
        } catch (const tf2::TransformException&) {}
        if (!pose_ok) {
            if (invalid_since.isZero()) invalid_since=ros::WallTime::now();
            if ((ros::WallTime::now()-invalid_since).toSec()>0.5) return stop_failed("Map-to-base pose unavailable or stale");
        } else {
            invalid_since=ros::WallTime();
            if (!speed.update(nearest)) return stop_failed("Cannot update corner speed");
        }
        ros::spinOnce();ros::WallDuration(0.10).sleep();
    }
    return stop_failed("Navigation timed out or task stopped");
}

} // namespace clean_toilet_robot
#endif
