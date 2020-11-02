#include <iostream>
#include "ros/ros.h"
#include "conversions.h"
#include "Planner.h"
#include "document.h"
#include "stringbuffer.h"
#include "prettywriter.h"
#include "conversions.h"
#include "CoveragePlanner.h"
#include "comm.hpp"
using namespace rapidjson;
using namespace std;
Planner::Planner()
{
    finalCoveragePathPub_ = nh_.advertise<nav_msgs::Path>("/satellite_map/utm_coverage_path_final", 2);
    carCmdPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    gpsCoveragePathFinalPub_ = nh_.advertise<std_msgs::String>("/satellite_map/gps_coverage_path_final", 1000);

    //bige map
    gpsclearpolygon_ = nh_.subscribe("/satellite_map/clear_polygon", 1, &Planner::clearPolygonCallback,this);
    gpsPolygonSub_ = nh_.subscribe("/satellite_map/gps_polygon", 5, &Planner::gpsPolygonMsgCallback, this);
    gpsGoalSub_ = nh_.subscribe("/satellite_map/gps_target", 1, &Planner::gpsTargetMsgCallback, this);
    webStartMsgSub_ = nh_.subscribe("/satellite_map/start", 1, &Planner::webStartMsgCallback, this);
    webStopMsgSub_ = nh_.subscribe("/satellite_map/stop", 1, &Planner::webStopMsgCallback, this);
    gpsCurrentPoseSub_ = nh_.subscribe("/sensor/gps", 1, &Planner::gpsCurrentPoseMsgCallback, this);
    pathDirectionSwitch_ = nh_.subscribe("/pathdirectionswitch", 1, &Planner::pathDirectionMsgCallback,this);
    pathCoveragemode_ = nh_.subscribe("/pathcoveragemode",  1, &Planner::pathCoverageMsgCallback, this);
    pathRotatemode_ = nh_.subscribe("/pathrotatemode",  1, &Planner::pathRotateMsgCallback, this);
    polygonmanualmode_ = nh_.subscribe("/manual_mode",  1, &Planner::manualGpsCallback, this);

    double path_width = getParam("~Path_width",3);
    double zoom_distance = getParam("~zoom_distance",1);
    new_planner = new coverage(path_width,zoom_distance);
    turningPointer_ = new Turning();
    trackingPointer_ = new Tracking(0.8, 0.36, 3); 
    
    resetPlannerParam();
}

Planner::~Planner()
{
    if (turningPointer_)
    {
        delete turningPointer_;
    }
    if (trackingPointer_)
    {
        delete trackingPointer_;
    }
}
void Planner::manualGpsCallback(const std_msgs::String &msg)
{
    ROS_INFO("manual GPS");
    manual_gps_mode = true;
    manual_point.points.resize(polygon_size);
    std::string s = msg.data.c_str();
    rapidjson::Document doc;
    if(!doc.Parse(s.data()).HasParseError())
    {
        if(doc.HasMember("shapes") && doc["shapes"].IsArray())
        {
            const rapidjson::Value &array = doc["shapes"];
            size_t len = array.Size();
            for(size_t i=0; i< len;i++)
            {
                const rapidjson::Value &object = array[i];
                size_t len2 = object.Size();
                for (size_t j =0;j < len2;j++)
                {
                    const rapidjson::Value &object2 = object[j];
                    double lat =0;
                    double lng =0;
                    if(object2.IsObject())
                    {
                        if (object2.HasMember("lat") && object2["lat"].IsDouble())
                            {
                                lat = object2["lat"].GetDouble();
                                cout << "lat=" << object2["lat"].GetDouble();
                            }
                            if (object2.HasMember("lng") && object2["lng"].IsDouble())
                            {
                                lng = object2["lng"].GetDouble();
                                cout << ", lng=" << object2["lng"].GetDouble() << endl;
                            }
                            manual_point.points[j].x = lat    ; //纬度
                            manual_point.points[j].y = lng   ; //经度
                    }
                }
            }
        }
    }
                      
    resetPlannerParam();
    new_planning();
    finalPath_ = turningPointer_->generateFinalPath(originalPath_, currentPose_);
    finalCoveragePathPub_.publish(finalPath_);
    pubGPSCoveragePathFinal(finalPath_);
    isGenerateFinalPathSuccessful_ = true;
    isReadyToRun_ = true;   
}
void Planner::pathRotateMsgCallback(const std_msgs::String &msg)
{
    ROS_INFO("rotate mode");
    new_planner->grass_mode = 1;
    originalPath_.poses.clear();
    finalPath_.poses.clear();
    new_planning();
    finalPath_ = turningPointer_->generateFinalPath(originalPath_, currentPose_);
    finalCoveragePathPub_.publish(finalPath_);
    pubGPSCoveragePathFinal(finalPath_);
    isGenerateFinalPathSuccessful_ = true;
}
void Planner::pathCoverageMsgCallback(const std_msgs::String &msg)
{
    ROS_INFO("coverage mode");
    new_planner->grass_mode = 0;
    originalPath_.poses.clear();
    finalPath_.poses.clear();
    new_planning();
    finalPath_ = turningPointer_->generateFinalPath(originalPath_, currentPose_);
    finalCoveragePathPub_.publish(finalPath_);
    pubGPSCoveragePathFinal(finalPath_);
    isGenerateFinalPathSuccessful_ = true;
}

void Planner::pathDirectionMsgCallback(const std_msgs::String &msg)
{
    ROS_INFO("switch path direction");
    new_planner->switch_index++;
    new_planner->switch_1stFlag = 1;
    if (new_planner->switch_index > (new_polygon.points.size()-1))
    {
        new_planner->switch_index = 0;
    }
    originalPath_.poses.clear();
    finalPath_.poses.clear();
    new_planning();
    finalPath_ = turningPointer_->generateFinalPath(originalPath_, currentPose_);
    finalCoveragePathPub_.publish(finalPath_);
    pubGPSCoveragePathFinal(finalPath_);
    isGenerateFinalPathSuccessful_ = true;
}
void Planner::clearPolygonCallback(const std_msgs::String &msg)
{
    ROS_INFO("clear polygon");
    resetPlannerParam();
}
void Planner::gpsPolygonMsgCallback(const std_msgs::String &msg)
{
    resetPlannerParam();
    manual_gps_mode = false;
    std::string s = msg.data.c_str();
    ROS_INFO("I heard: [%s]", msg.data.c_str());
    rapidjson::Document doc;
    if (!doc.Parse(s.data()).HasParseError())
    {
        if (doc.HasMember("shapes") && doc["shapes"].IsArray())
        {
            const rapidjson::Value &array = doc["shapes"];
            size_t len = array.Size();
            for (size_t i = 0; i < len; i++)
            {
                const rapidjson::Value &object = array[i];
                size_t len2 = object.Size();

                for (size_t j = 0; j < len2; j++)
                {
                    const rapidjson::Value &object2 = object[j];
                    double lat = 0.0;
                    double lng = 0.0;
                    if (object2.IsObject())
                    {
                        cout << "ObjectArray[" << j << "]: ";
                        if (object2.HasMember("lat") && object2["lat"].IsDouble())
                        {
                            lat = object2["lat"].GetDouble();
                            cout << "lat=" << object2["lat"].GetDouble();
                        }
                        if (object2.HasMember("lng") && object2["lng"].IsDouble())
                        {
                            lng = object2["lng"].GetDouble();
                            cout << ", lng=" << object2["lng"].GetDouble() << endl;
                        }
                        double UTM_N;
                        double UTM_E;
                        std::string zone;
                        gps_common::LLtoUTM(lat, lng, UTM_N, UTM_E, zone);
                        utm_zone = zone;
                        geometry_msgs::Point32 pBuf;
                        pBuf.x = UTM_E;
                        pBuf.y = UTM_N;

                        pBuf.x = UTM_E ;
                        pBuf.y = UTM_N ;
                        new_polygon.points.push_back(pBuf);
                    }
                }
            }
        }
    }
    ROS_INFO("Start to plan");
    if (!isNavToPointMode_)
    {
        new_planning();
        finalPath_ = turningPointer_->generateFinalPath(originalPath_, currentPose_);
        finalCoveragePathPub_.publish(finalPath_);
        pubGPSCoveragePathFinal(finalPath_);
        isGenerateFinalPathSuccessful_ = true;
    }
    isReadyToRun_ = true;
    std::cout << isReadyToRun_ << "/" << isNavToPointMode_ << "/" << isGenerateFinalPathSuccessful_ << std::endl;
}
void Planner::gpsTargetMsgCallback(const std_msgs::String &msg)
{
    ROS_INFO("Nav to one point mode!");
    goal_ = transformTargetFromGps2Utm(msg);
}
void Planner::gpsCurrentPoseMsgCallback(const sensor_msgs::NavSatFix &msg)
{
    if (msg.status.status != 4)
    {
        ROS_INFO("Missed gps data");
        return;
    }

    currentPose_ = transformCurrentPoseFromGps2Utm(msg);
    if (!isReadyToRun_)
    {
        return;
    }
    if (isNavToPointMode_)
    {

        if (!trackingPointer_->run(currentPose_, goal_))
        {
            resetPlannerParam();
        }
        return;
    }
    else if (isGenerateFinalPathSuccessful_)
    {
        if (!(trackingPointer_->run(currentPose_, finalPath_)))
        {
            resetPlannerParam();
        }
    }
}
void Planner::pubGPSCoveragePathFinal(const nav_msgs::Path &msg)
{
    int pathLength = msg.poses.size();
    Document doc;
    Document::AllocatorType &allocator = doc.GetAllocator();
    doc.SetObject();
    Value array(kArrayType);
    for (int i = 0; i < pathLength; i++)
    {
        Value item(kObjectType);
        double utm_e = msg.poses[i].pose.position.x;
        double utm_n = msg.poses[i].pose.position.y;
        double gps_lat;
        double gps_lng;
        gps_common::UTMtoLL(utm_n, utm_e, utm_zone, gps_lat, gps_lng);
        item.AddMember("lat", gps_lat, allocator);
        item.AddMember("lng", gps_lng, allocator);
        if (i % 10 == 0)
        {
            array.PushBack(item, allocator);
        }
        //cout << "gps:" << fixed<<setprecision(16)<<gps_lat<<"----"<< gps_lng << endl;
    }
    doc.AddMember("shapes", array, allocator);
    StringBuffer stringBuf;
    PrettyWriter<StringBuffer> writer(stringBuf);
    doc.Accept(writer);
    std_msgs::String gps;
    gps.data = stringBuf.GetString();
    cout << "gps:" << gps << endl;
    ROS_INFO("publish GPS path");
    gpsCoveragePathFinalPub_.publish(gps);
}

void Planner::webStartMsgCallback(const std_msgs::String &msg)
{
    ROS_INFO("Start to run");
    trackingPointer_->stop_signal = 1;
}
void Planner::webStopMsgCallback(const std_msgs::String &msg)
{
    ROS_INFO("Stop and init");
    trackingPointer_->stop_signal = 0;
}
geometry_msgs::PoseStamped Planner::transformTargetFromGps2Utm(std_msgs::String gps_target)
{
    std::string s = gps_target.data.c_str();
    ROS_INFO("I heard: [%s]", gps_target.data.c_str());
    rapidjson::Document doc;
    double lat = 0.0;
    double lng = 0.0;
    if (!doc.Parse(s.data()).HasParseError())
    {
        const rapidjson::Value &json_lat = doc["lat"];
        lat = json_lat.GetDouble();
        cout << "lat:" << lat << endl;
        const rapidjson::Value &json_lng = doc["lng"];
        lng = json_lng.GetDouble();
        cout << "lng:" << lng << endl;
    }
    double N;
    double E;
    geometry_msgs::PoseStamped target;
    std::string ss;
    gps_common::LLtoUTM(lat, lng, N, E, ss);
    target.pose.position.x = E;
    target.pose.position.y = N;
    cout << " utmzone " << ss <<endl;
    cout << "target_x--target_y" << target.pose.position.x << "--" << target.pose.position.y << endl;
    return target;
}
geometry_msgs::PoseStamped Planner::transformCurrentPoseFromGps2Utm(sensor_msgs::NavSatFix gps_current_pose)
{
    geometry_msgs::PoseStamped curPose;
    double N, E;
    double heading;
    std::string ss;
    gps_common::LLtoUTM(gps_current_pose.latitude, gps_current_pose.longitude, N, E, ss);
    //    heading = angleDiff(PI / 2, msg.altitude);
    curPose.header.frame_id = "map";
    curPose.pose.position.x = E;
    curPose.pose.position.y = N;
    curPose.pose.orientation = tf::createQuaternionMsgFromYaw((90.0 - gps_current_pose.altitude) / 180.0 * M_PI);
    return curPose;
}
void Planner::resetPlannerParam()
{
    isGenerateFinalPathSuccessful_ = false;
    isNavToPointMode_ = false;
    isReadyToRun_ = false;
    new_polygon.points.clear();
    new_start.pose.position.x = 0;
    new_start.pose.position.y = 0;
    originalPath_.poses.clear();
    finalPath_.poses.clear();
    originalPath_.header.frame_id = "map";
    finalPath_.header.frame_id = "map";
    cmd_vel_.linear.x = 0;
    cmd_vel_.angular.z = 0;
    new_planner->switch_index = 0;
    carCmdPub_.publish(cmd_vel_);
    finalCoveragePathPub_.publish(finalPath_);
}
void Planner::new_planning()
{
    cout << "test" << endl;   
    nav_msgs::Path new_path;
    new_start.pose.position.x = currentPose_.pose.position.x;
    new_start.pose.position.y = currentPose_.pose.position.y;
    currentPose_.pose.position.x = goal_.pose.position.x;
    currentPose_.pose.position.y = goal_.pose.position.y;
    new_start.pose.position.x = goal_.pose.position.x;
    new_start.pose.position.y = goal_.pose.position.y;
    if (manual_gps_mode == true)
    {  
        new_polygon.points.clear();
        geometry_msgs::Point32 tmp_point;
        for(int i = 0;i < polygon_size;i++)
        {
            double utm_n, utm_e;
            double lat, lng;
            lat = manual_point.points[i].x;
            lng = manual_point.points[i].y;
            std::string zone;
            gps_common::LLtoUTM(lat, lng, utm_n, utm_e, zone);
            utm_zone = zone;
            tmp_point.x = utm_e;
            tmp_point.y = utm_n;
            new_polygon.points.push_back(tmp_point);
        }
    }
    new_path = new_planner->generateOriginalCoveragePath(new_polygon,new_start);
    geometry_msgs::PoseStamped pointBuf;
    pointBuf.header.frame_id = "map";
    originalPath_.header.frame_id = "map";
    for (size_t i = 0; i < new_path.poses.size(); i++)
    {
        pointBuf.pose.position.x = new_path.poses[i].pose.position.x ;
        pointBuf.pose.position.y = new_path.poses[i].pose.position.y ;
        pointBuf.pose.position.z = new_path.poses[i].pose.position.z ;
        originalPath_.poses.push_back(pointBuf);
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planner");
    Planner pl;
    std::cout << "Planner node start" << std::endl;
    ros::spin();
    return 0;
}
