#ifndef TURNING_H_
#define TURNING_H_

#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <vector>
#include "MathCommon.h"
struct CarModel
{
    /* data */
    double carLength;
    double carWidth;
    double plowWidth;
    double taMax;
    double turnRadiusMin;
    double vMax;
    double wMax;

    CarModel() ; 

};

class Turning
{
public:
    Turning(/* args */);
    nav_msgs::Path generateFinalPath(nav_msgs::Path originalPath, geometry_msgs::PoseStamped current_pose);

private:
    void addOrientationToEveryPoint(nav_msgs::Path &originpath);
    nav_msgs::Path calculateFishTailPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car);
    nav_msgs::Path calculateBulbPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car);
    nav_msgs::Path calculateBowPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car);
    nav_msgs::Path calculateTurningPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car);
    const CarModel carModel_;
    geometry_msgs::PoseStamped end_;
};

#endif