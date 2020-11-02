#include <iostream>
#include "Tracking.h"
#include "comm.hpp"
#define PI 3.1415926
Tracking::Tracking(double max_linear_velocity, double max_angular_velocity, double distance_threshold) : maxLinearVel_(max_linear_velocity), maxAngularVel_(max_angular_velocity), threshold_(distance_threshold)
{
    cmdPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    lookaheadPosePub_ = nh_.advertise<visualization_msgs::Marker>("lookahead_pose", 1);
    reset();
}

void Tracking::reset()
{
    pathIndex_ = 0;
    lookahead_ = 1;

    Kv_ = getParam<float>("~kv",0.1);
    Kp_ = getParam<float>("~kp",0.1);
    Kd_ = getParam<float>("~kd",0.1);
    error_ = 0;
    curPath_.poses.clear();
    cmd_vel_.angular.z = 0;
    cmd_vel_.linear.x = 0;
    cmdPublish();
}

void Tracking::cmdPublish()
{ 
    if (stop_signal==1)
    {
      cmdPub_.publish(cmd_vel_);
    }
    else
    {
      cmd_vel_.angular.z = 0;
      cmd_vel_.linear.x = 0;
      cmdPub_.publish(cmd_vel_);
    }
    
}

bool Tracking::run(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal)
{
    curPose_ = current_pose;
    curHeading_ = quaterniond2Euler(curPose_);
    if (!isTargetReached(current_pose, goal))
    {
        computeCmdVelForOnePoint(curPose_, goal);
        return true;
    }
    else
    {
        reset();
        ROS_INFO("Goal Reached!");
        cmdPublish();
        return false;
    }
}

bool Tracking::run(geometry_msgs::PoseStamped current_pose, nav_msgs::Path tracking_path)
{
    curPose_ = current_pose;
    if (getDistance(end_, tracking_path.poses[tracking_path.poses.size() - 1]) > 0.2)
    {
        reset();
        curPath_ = tracking_path;
        end_ = tracking_path.poses[tracking_path.poses.size() - 1];
    }
    curHeading_ = quaterniond2Euler(curPose_);
    if (!curPath_.poses.empty())
    {
        if (!isTargetReached(curPose_, end_))
        {
            updateLookaheadPoint(curPose_);
            computeCmdVel(curPose_, pointLookahead_);
            return true;
        }
        else
        {
            reset();
            ROS_INFO("Goal Reached!");
            cmdPublish();
            return false;
        }
    }
    return false;
}

bool Tracking::updateLookaheadPoint(geometry_msgs::PoseStamped poseCur)
{
    if (pathIndex_ == curPath_.poses.size() - 1)
    {
        return false;
    }
    pointLookahead_ = curPath_.poses[pathIndex_];
    double dist = getDistance(poseCur, pointLookahead_);
    while (dist < lookahead_)
    {
        pathIndex_++;
        pointLookahead_ = curPath_.poses[pathIndex_];
        dist = getDistance(poseCur, pointLookahead_);
        if (pathIndex_ == curPath_.poses.size() - 1)
        {
            return true;
        }
    }
    visualization_msgs::Marker points;
    geometry_msgs::Point p;
    p.x = pointLookahead_.pose.position.x;
    p.y = pointLookahead_.pose.position.y;
    points.header.frame_id = "map";
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.b = 1.0;
    points.color.a = 1.0;
    points.points.push_back(p);
    lookaheadPosePub_.publish(points);
    return true;
}

bool Tracking::computeCmdVel(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped goalCur)
{
    double curXY[2] = {0};
    double followXY[2] = {0};
    double hx, hy;
    curXY[0] = poseCur.pose.position.x;
    curXY[1] = poseCur.pose.position.y;
    followXY[0] = goalCur.pose.position.x;
    followXY[1] = goalCur.pose.position.y;
    double fx = followXY[0];
    double fy = followXY[1];
    double beta = angleDiff(quaterniond2Euler(goalCur), quaterniond2Euler(poseCur));
    transformFromMap2Car(fx, fy, curXY[0], curXY[1], addTwoAngle(quaterniond2Euler(poseCur), -PI / 2));
    if ((fy < 0) && (fabs(beta) < (PI / 2)))
    {
        backward(poseCur, goalCur);
        ROS_INFO("back car!");
    }
    else
    {
        lookahead_ = getParam<double>("~lookahead",3.0);
        double linV = Kv_ * getDistance(curXY, followXY); 

        ROS_INFO("lookahead distance = %lf",getDistance(curXY,followXY));

        double angle = angleDiff(atan2(followXY[1] - curXY[1], followXY[0] - curXY[0]), curHeading_);
         double angV = -1 *( Kp_ * angle + Kd_*(angle - error_)) ;
         error_ = angle;
        cmd_vel_.linear.x = fabs(linV) < maxLinearVel_ ? linV : ((linV / fabs(linV)) * maxLinearVel_);
        cmd_vel_.angular.z = fabs(angV) < maxAngularVel_ ? angV : ((angV / fabs(angV)) * maxAngularVel_);
    }
    cmdPublish();
}

bool Tracking::computeCmdVelForOnePoint(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped goalCur)
{
    double curXY[2] = {0};
    double followXY[2] = {0};
    curXY[0] = poseCur.pose.position.x;
    curXY[1] = poseCur.pose.position.y;
    followXY[0] = goalCur.pose.position.x;
    followXY[1] = goalCur.pose.position.y;
    double fx = followXY[0];
    double fy = followXY[1];
    transformFromMap2Car(fx, fy, curXY[0], curXY[1], addTwoAngle(quaterniond2Euler(poseCur), -PI / 2));
    double linV = Kv_ * getDistance(curXY, followXY);
    double angle = angleDiff(atan2(followXY[1] - curXY[1], followXY[0] - curXY[0]), curHeading_);
    double angV = -1 * Kp_ * angle;
    cmd_vel_.linear.x = fabs(linV) < maxLinearVel_ ? linV : ((linV / fabs(linV)) * maxLinearVel_);
    cmd_vel_.angular.z = fabs(angV) < maxAngularVel_ ? angV : ((angV / fabs(angV)) * maxAngularVel_);
    cmdPublish();
}

void Tracking::backward(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped goalCur)
{
    double goal[2] = {0};
    double distance = 0;
    double theta = 0;
    double Kv = 0.7;
    goal[0] = goalCur.pose.position.x;
    goal[1] = goalCur.pose.position.y;
    lookahead_ = 1;
    if (isTargetReached(curPose_, end_))
    {
        reset();
        return;
    }
    else
    {
        distance = sqrt(pow((poseCur.pose.position.x - goal[0]), 2) + pow((poseCur.pose.position.y - goal[1]), 2));
        double linV = -1 * Kv * distance;
        theta = angleDiff(quaterniond2Euler(poseCur), atan2(poseCur.pose.position.y - goal[1], poseCur.pose.position.x - goal[0]));
        double beta = angleDiff(quaterniond2Euler(poseCur), quaterniond2Euler(goalCur));
        double angV = -1 * (0.8 * theta + 0.6 * beta); //  0.8/0.4
        cmd_vel_.linear.x = fabs(linV) < maxLinearVel_ ? linV : ((linV / fabs(linV)) * maxLinearVel_);
        cmd_vel_.angular.z = fabs(angV) < maxAngularVel_ ? angV : ((angV / fabs(angV)) * maxAngularVel_);
    }
}

bool Tracking::isTargetReached(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal)
{
    double d = getDistance(current_pose, goal);
    ROS_INFO("distance = %lf", d);
    if (d <= threshold_)
    {
        reset();
        return true;
    }
    else
        return false;
}
