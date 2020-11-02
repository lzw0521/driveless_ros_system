#ifndef PLANNER_H_
#define PLANNER_H_

#include "Turning.h"
#include "Tracking.h"
#include "CoveragePlanner.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PolygonStamped.h"
#include "conversions.h"
#include "rapidjson/document.h"

class Planner
{
public:
    Planner();
    ~Planner();

    //real time
    void gpsPolygonMsgCallback(const std_msgs::String &msg);
    void gpsTargetMsgCallback(const std_msgs::String &msg);
    void gpsCurrentPoseMsgCallback(const sensor_msgs::NavSatFix &msg);
    void webStartMsgCallback(const std_msgs::String &msg);
    void webStopMsgCallback(const std_msgs::String &msg);
    void clearPolygonCallback(const std_msgs::String &msg);
    void pathDirectionMsgCallback(const std_msgs::String &msg);
    void pathCoverageMsgCallback(const std_msgs::String &msg);
    void pathRotateMsgCallback(const std_msgs::String &msg);
    void manualGpsCallback(const std_msgs::String &msg);

    //operation to pathplan
    void resetPlannerParam();
    void new_planning();
    geometry_msgs::PolygonStamped transformPolygonFromGps2Utm(std_msgs::String gps_polygon);
    void publishStringPath(const nav_msgs::Path& msg);
    geometry_msgs::PoseStamped transformTargetFromGps2Utm(std_msgs::String gps_target);
    geometry_msgs::PoseStamped transformCurrentPoseFromGps2Utm(sensor_msgs::NavSatFix gps_current_pose);
    void pubGPSCoveragePathFinal(const nav_msgs::Path& msg);

    //new_planning
    geometry_msgs::PoseStamped new_start;
    geometry_msgs::Polygon new_polygon;

    //gps打点
    bool manual_gps_mode = 0; //置1为打点模式
    int polygon_size = 4;  //打点个数
    geometry_msgs::Polygon manual_point;
    std::string utm_zone;

private:
    ros::NodeHandle nh_;
    ros::ServiceClient plannerClient_;
    ros::Publisher originalCoveragePathPub_;
    ros::Publisher finalCoveragePathPub_;
    ros::Publisher carCmdPub_;
    ros::Publisher gpsCoveragePathFinalPub_;

    // bige map
    ros::Subscriber gpsPolygonSub_;
    ros::Subscriber gpsGoalSub_;
    ros::Subscriber gpsCurrentPoseSub_;
    ros::Subscriber webStartMsgSub_;
    ros::Subscriber webStopMsgSub_;
    ros::Subscriber gpsclearpolygon_;
    ros::Subscriber pathDirectionSwitch_;
    ros::Subscriber pathCoveragemode_;
    ros::Subscriber pathRotatemode_;
    ros::Subscriber polygonmanualmode_;
    Turning *turningPointer_;
    Tracking *trackingPointer_;
    coverage *new_planner;
    geometry_msgs::PoseStamped currentPose_;
    geometry_msgs::PoseStamped goal_;
    geometry_msgs::Twist cmd_vel_;
    nav_msgs::Path originalPath_;
    nav_msgs::Path finalPath_;
 
    bool isGenerateFinalPathSuccessful_;
    bool isNavToPointMode_;
    bool isReadyToRun_; 
};

#endif
