#ifndef COVERAGEPLANNER_H_
#define COVERAGEPLANNER_H_

#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Path.h"
#include <Eigen/Dense>

struct Line
{
    Eigen::Vector2f vector;
    Eigen::Vector2f normalVector;
    geometry_msgs::Point32 point;
    int index_1, index_2; //向量A-B
    Line(Eigen::Vector2f d, geometry_msgs::Point32 p, int index1, int index2) : vector(d), point(p),
                                                                                index_1(index1), index_2(index2)
    {
        vector.normalize();
        normalVector[0] = -vector[1];
        normalVector[1] = vector[0];
    }
    Line(Eigen::Vector2f d, geometry_msgs::Point32 p) : vector(d), point(p)
    {
        vector.normalize();
        normalVector[0] = -vector[1];
        normalVector[1] = vector[0];
    }
    Line() {}
};
struct Segment
{
    geometry_msgs::Point32 endpointA;
    geometry_msgs::Point32 endpointB;
    Segment(geometry_msgs::Point32 a, geometry_msgs::Point32 b) : endpointA(a), endpointB(b) {}
    Segment() {}
};

class coverage
{
public:
    coverage(double pathWidth, double zoomDistance);


    //
    nav_msgs::Path generateOriginalCoveragePath(geometry_msgs::Polygon polygon, geometry_msgs::PoseStamped start,
                                                double theta, bool useStartHeading);
    // 入口
    nav_msgs::Path generateOriginalCoveragePath(geometry_msgs::Polygon polygon, geometry_msgs::PoseStamped start);
    nav_msgs::Path calculateRotatePath(nav_msgs::Path path);

    geometry_msgs::Polygon calculateSubPolygon(geometry_msgs::Polygon polygon);
    geometry_msgs::Point32 calculateVertexOfSubPolygon(geometry_msgs::Point32 vertex,
                                                       geometry_msgs::Point32 neighbor1, geometry_msgs::Point32 neighbor2);
    bool calculateIntersectionsOfLineAndPolygon(std::vector<Segment> segments, Line currentLine,
                                                geometry_msgs::PoseStamped &a, geometry_msgs::PoseStamped &b);
    bool calculateIntersectionOfLineAndSegment(Segment segment,Line currentLine, geometry_msgs::PoseStamped &c);
    std::vector<Segment> generateSegmentsFromPolygon(geometry_msgs::Polygon polygon);
    Line generateBaseLineFromSubPolygon(geometry_msgs::Polygon subPolygon);
    Line generateBaseLineFromStartPose(geometry_msgs::PoseStamped start, double theta, bool useStartHeading);
    Line generateNewLineToCalculatePath(Line pre, int polygonDirection, double pathWidth);
    int calculatePolygonDirection(geometry_msgs::Polygon subPolygon, Line baseLine);
    double getSignedDistanceOfPoint2Line(geometry_msgs::Point32 vertex, Line baseLine);

    double calculateAngleOfTwoVector(Eigen::Vector2f v1, Eigen::Vector2f v2);
    int switch_index ;
    int switch_1stFlag ;
    bool grass_mode = 0 ; //路径模式切换，置1为回型
private:
    // std::vector<Vertex2f> vertex_;
    ros::NodeHandle nh_;
    ros::Publisher subPolygonPub_;
    ros::Publisher polygonPub_;
    ros::Publisher coveragePathPub_;

    nav_msgs::Path originalCoveragePath_;
    geometry_msgs::PolygonStamped polygon_;
    geometry_msgs::PolygonStamped subPolygon_;
    geometry_msgs::PoseStamped startPose_;
    std::vector<Segment> polygonSegments_;
    double pathWidth_;
    double zoomDistance_;  //正-缩,负-放
    int polygonDirection_; //1-逆时针,2-顺时针
    Line baseLine_;
};

#endif