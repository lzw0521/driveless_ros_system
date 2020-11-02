#ifndef TRACKING_H_
#define TRACKING_H_

#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <vector>
#include "MathCommon.h"



class Tracking
{
public:
    Tracking(double max_linear_velocity, double max_angular_velocity, double distance_threshold);
    bool isTargetReached(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal);
    void reset();
    bool updateLookaheadPoint(geometry_msgs::PoseStamped poseCur);
    bool computeCmdVel(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped goalCur);
    bool computeCmdVelForOnePoint(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped goalCur);
    void backward(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped goalCur);
    bool run(geometry_msgs::PoseStamped current_pose, nav_msgs::Path tracking_path);
    bool run(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal);
    geometry_msgs::Twist getCmd()
    {
        return cmd_vel_;
    }
    geometry_msgs::PoseStamped &getCurrentPose();  
    int stop_signal;
private:
    void cmdPublish();
    ros::NodeHandle nh_;
    ros::Publisher cmdPub_;
    ros::Publisher lookaheadPosePub_;
    geometry_msgs::Twist cmd_vel_;
    geometry_msgs::PoseStamped curPose_;
    geometry_msgs::PoseStamped end_;
    geometry_msgs::PoseStamped pointLookahead_;
    nav_msgs::Path curPath_;
    unsigned int pathIndex_;
    double curHeading_;
    double lookahead_;
    double maxLinearVel_, maxAngularVel_;
    double threshold_;
    double Kv_, Ki_, Kh_, Kb_,Kd_,error_,Kp_;
};

#endif