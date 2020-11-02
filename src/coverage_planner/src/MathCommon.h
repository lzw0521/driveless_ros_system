#ifndef MATHCOMMON_H_
#define MATHCOMMON_H_

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "tf/transform_datatypes.h" //转换函数头文件

void transformFromCar2Map(double &x, double &y, double a, double b, double theta);
void transformFromMap2Car(double &x, double &y, double a, double b, double theta);
double getDistance(double poseCur[2], double lookahead[2]);
double getDistance(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped lookahead);
double getAngle(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b);
double addTwoAngle(double a, double b);
double angleDiff(double a, double b);
double quaterniond2Euler(geometry_msgs::PoseStamped data);
void addTwoPath(nav_msgs::Path &final, nav_msgs::Path turn);
void threeAxisRot(double r11, double r12, double r21, double r31, double r32, double *euler);
void intersectionOf2Circles(double a1, double b1, double R1, double a2, double b2, double R2, double &x, double &y);
void interpolateToPath(nav_msgs::Path &path, double interval);
void interpolateToPath2(nav_msgs::Path &path, double interval,geometry_msgs::PoseStamped _end);
#endif