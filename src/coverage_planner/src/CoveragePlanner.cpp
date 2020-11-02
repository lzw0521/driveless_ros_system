#include <cmath>
#include <iostream>
#include "CoveragePlanner.h"
#include "MathCommon.h"

using namespace std;

coverage::coverage(double pathWidth, double zoomDistance) : pathWidth_(pathWidth), zoomDistance_(zoomDistance)
{
    subPolygonPub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/subPolygon", 1);
    polygonPub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/Polygon", 1);
    coveragePathPub_ = nh_.advertise<nav_msgs::Path>("/original_path", 2);
}


nav_msgs::Path coverage::generateOriginalCoveragePath(geometry_msgs::Polygon polygon, geometry_msgs::PoseStamped start,
                                                      double theta, bool useStartHeading)
{
    nav_msgs::Path coveragePath;
    if (polygon.points.size() < 3)
    {
        ROS_INFO("Invalid polygon input");
    }
    startPose_ = start;
    polygon_.polygon = polygon;
    polygonDirection_ = theta;

    subPolygon_.polygon = calculateSubPolygon(polygon);
    return coveragePath;
}

nav_msgs::Path coverage::generateOriginalCoveragePath(geometry_msgs::Polygon polygon, geometry_msgs::PoseStamped start)
{
    nav_msgs::Path coveragePath;
    nav_msgs::Path rotate_path;
    Line nextLine;

    if (polygon.points.size() < 3)
    {
        ROS_INFO("Invalid polygon input");
        return coveragePath;
    }
    cout << "start:" << start << endl;
    polygon_.polygon = polygon;
    subPolygon_.polygon = calculateSubPolygon(polygon);
    polygonSegments_ = generateSegmentsFromPolygon(subPolygon_.polygon);
    baseLine_ = generateBaseLineFromSubPolygon(subPolygon_.polygon);
    polygonDirection_ = calculatePolygonDirection(subPolygon_.polygon, baseLine_);
    nextLine = baseLine_;
    nextLine = generateNewLineToCalculatePath(nextLine, polygonDirection_, pathWidth_);
    geometry_msgs::PoseStamped a, b, c;
    geometry_msgs::PoseStamped previousPoint = start;
    double d1, d2;
    while (calculateIntersectionsOfLineAndPolygon(polygonSegments_, nextLine, a, b))
    {
        d1 = getDistance(a, previousPoint);
        d2 = getDistance(b, previousPoint);
        if (d1 <= d2)
        {
            coveragePath.poses.push_back(a);
            coveragePath.poses.push_back(b);
            previousPoint = b;
        }
        else
        {
            coveragePath.poses.push_back(b);
            coveragePath.poses.push_back(a);
            previousPoint = a;
        }
        nextLine = generateNewLineToCalculatePath(nextLine, polygonDirection_, pathWidth_);
    }
    cout << "coveragePath.size = " << coveragePath.poses.size() << endl;
    coveragePath.header.frame_id = "map";
    int path_size = coveragePath.poses.size();
    geometry_msgs::PoseStamped path_start, path_end;
    path_start = coveragePath.poses[0];
    path_end = coveragePath.poses[path_size-1]; 
    double s1, s2;
    s1 = sqrt(pow((path_start.pose.position.x - start.pose.position.x), 2) + pow((path_start.pose.position.y - start.pose.position.y), 2));
    s2 = sqrt(pow((path_end.pose.position.x - start.pose.position.x), 2) + pow((path_end.pose.position.y - start.pose.position.y), 2));
    nav_msgs::Path final_coveragepath;
    final_coveragepath.header.frame_id = "map";
    if(s1 - s2>0)
    {
    cout << "s1>s2" << endl;
      final_coveragepath.poses.resize(path_size);
      for(int i = 0; i < path_size; i++)
      {
        final_coveragepath.poses[i] = coveragePath.poses[path_size-1-i];
      }
        if(grass_mode == true)
       {
         rotate_path = calculateRotatePath(final_coveragepath);
         coveragePathPub_.publish(rotate_path);
         return rotate_path;
       }
        
      return final_coveragepath;
    }
    if(grass_mode == true)
    {
        rotate_path = calculateRotatePath(coveragePath);
        coveragePathPub_.publish(rotate_path);
        return rotate_path;
    }
    return coveragePath;
}

nav_msgs::Path coverage::calculateRotatePath(nav_msgs::Path path)
{
    nav_msgs::Path path_rotate;
    path_rotate.header.frame_id = "map";
    int pathsize = path.poses.size();
    cout << "pathsize = " << pathsize << endl;
    path_rotate.poses.resize(pathsize);
    if(pathsize % 2 == 0 && (pathsize/2)%2 == 0)//处理偶数行全局路径
    { 
      for(int i = 1; i <= pathsize / 2; i++) //i代表行号，一行有两个点
      {
          if (i%2 != 0)//奇数行代表原路径前半段
          { 
             int k1,k2;
             k1 = 2*(i-1);
             k2 = 2*(((i+1)/2)-1);
             if (((i+1)/2)%2 !=0)
             { 
                path_rotate.poses[k1] = path.poses[k2];
                path_rotate.poses[k1+1] = path.poses[k2+1];
             }
             else
             {
                path_rotate.poses[k1] = path.poses[k2+1];
                path_rotate.poses[k1+1] = path.poses[k2];
             }
          }
          else //偶数行代表后半段
          {  
            int k1,k2;
            k1 = 2*(i-1);
            k2 = 2*(((2-i)/2+pathsize/2)-1);
            if(((2-i)/(-2)+pathsize/2)%2 !=0)
            {
               path_rotate.poses[k1] = path.poses[k2+1];
               path_rotate.poses[k1+1] = path.poses[k2];
            }
            else
            {
               path_rotate.poses[k1] = path.poses[k2];
               path_rotate.poses[k1+1] = path.poses[k2+1];
            }                          
          }
      }
    }
    else if(pathsize % 2 == 0 && (pathsize/2)%2 != 0)//处理奇数行全局路径
    { 
      for(int i = 1; i <= pathsize / 2; i++) 
      { 
          if(((pathsize/2-1)/2)%2 == 0)
          {  
            path_rotate.poses[pathsize-2] = path.poses[2*(((pathsize/2)-1)/2+1)-1];//小
            path_rotate.poses[pathsize-1] = path.poses[2*(((pathsize/2)-1)/2+1)-2];//大
          }
          else
          {
            path_rotate.poses[pathsize-2] = path.poses[2*(((pathsize/2)-1)/2+1)-2];//大
            path_rotate.poses[pathsize-1] = path.poses[2*(((pathsize/2)-1)/2+1)-1];//小
          }      
          if (i%2 != 0 )//奇数行代表原路径前半段
          { 
             int k1,k2;
             k1 = 2*(i-1);
             k2 = 2*(((i+1)/2)-1);
             if (((i+1)/2)%2 !=0)
             { 
                path_rotate.poses[k1] = path.poses[k2];
                path_rotate.poses[k1+1] = path.poses[k2+1];
             }
             else
             {
                path_rotate.poses[k1] = path.poses[k2+1];
                path_rotate.poses[k1+1] = path.poses[k2];
             }
          }
          else//偶数行代表后半段
          {  
            int k1,k2;
            k1 = 2*(i-1);
            k2 = 2*(((2-i)/2+pathsize/2)-1);
            if(((2-i)/(-2)+pathsize/2)%2 !=0)
            {
               path_rotate.poses[k1] = path.poses[k2+1];
               path_rotate.poses[k1+1] = path.poses[k2];
            }
            else
            {
               path_rotate.poses[k1] = path.poses[k2];
               path_rotate.poses[k1+1] = path.poses[k2+1];
            }                          
          }
      }
    }
    return path_rotate;
}
bool coverage::calculateIntersectionsOfLineAndPolygon(std::vector<Segment> segments, Line currentLine,
                                                      geometry_msgs::PoseStamped &a, geometry_msgs::PoseStamped &b)
{
    a.header.frame_id = "map";
    b.header.frame_id = "map";
    geometry_msgs::PoseStamped c;
    std::vector<geometry_msgs::PoseStamped> buffer;
    for (size_t i = 0; i < segments.size(); i++)
    {
        if (calculateIntersectionOfLineAndSegment(segments[i], currentLine, c))
        {
            buffer.push_back(c);
        }
    }
    if (buffer.empty())
    {
        return false;
    }
    else
    {
        cout << "buffer.size = " << buffer.size() << endl;
        a = buffer[0];
        b = buffer[1];
        return true;
    }
}

bool coverage::calculateIntersectionOfLineAndSegment(Segment segment, Line currentLine, geometry_msgs::PoseStamped &c)
{
    double d1 = getSignedDistanceOfPoint2Line(segment.endpointA, currentLine);
    double d2 = getSignedDistanceOfPoint2Line(segment.endpointB, currentLine);
    if ((d1 == 0))
    {
        c.pose.position.x = segment.endpointA.x;
        c.pose.position.y = segment.endpointA.y;
        return true;
    }
    else if (d2 == 0)
    {
        c.pose.position.x = segment.endpointB.x;
        c.pose.position.y = segment.endpointB.y;
        return true;
    }
    else if (d1 * d2 < 0)
    {
        double D1 = fabs(d1);
        double D2 = fabs(d2);
        double k = D1 / (D1 + D2);
        c.pose.position.x = k * (segment.endpointB.x - segment.endpointA.x) + segment.endpointA.x;
        c.pose.position.y = k * (segment.endpointB.y - segment.endpointA.y) + segment.endpointA.y;
        return true;
    }
    else
    {
        return false;
    }
}

Line coverage::generateNewLineToCalculatePath(Line pre, int polygonDirection, double pathWidth)
{
    geometry_msgs::Point32 linePoint;
    if (polygonDirection == 1)
    {
        linePoint.x = pre.point.x + pre.normalVector[0] * pathWidth;
        linePoint.y = pre.point.y + pre.normalVector[1] * pathWidth;
    }
    else if (polygonDirection == 2)
    {
        linePoint.x = pre.point.x - pre.normalVector[0] * pathWidth;
        linePoint.y = pre.point.y - pre.normalVector[1] * pathWidth;
    }
    Line newLine(pre.vector, linePoint);
    return newLine;
}

Line coverage::generateBaseLineFromSubPolygon(geometry_msgs::Polygon subPolygon)
{
    double index_1, index_2;
    if (switch_index == 0 && switch_1stFlag == 0)
    {
        double distanceOfTwoVertex;
        double dMax = 0;
       // double index_1, index_2;
        for (size_t i = 0; i < subPolygon.points.size() - 1; i++)
        {
            distanceOfTwoVertex = sqrt(pow(subPolygon.points[i + 1].y - subPolygon.points[i].y, 2) +
                                    pow(subPolygon.points[i + 1].x - subPolygon.points[i].x, 2));
            if (distanceOfTwoVertex > dMax)
            {
                dMax = distanceOfTwoVertex;
                index_1 = i;
                index_2 = i + 1;
            }
        }
        distanceOfTwoVertex = sqrt(pow(subPolygon.points[subPolygon.points.size() - 1].y - subPolygon.points[0].y, 2) +
                                pow(subPolygon.points[subPolygon.points.size() - 1].x - subPolygon.points[0].x, 2));
        if (distanceOfTwoVertex > dMax)
        {
            dMax = distanceOfTwoVertex;
            index_1 = subPolygon.points.size() - 1;
            index_2 = 0;
        }
        Eigen::Vector2f baseVector(subPolygon.points[index_2].x - subPolygon.points[index_1].x,
                                subPolygon.points[index_2].y - subPolygon.points[index_1].y);
        Line baseLine(baseVector, subPolygon.points[index_1], index_1, index_2);
        baseLine_ = baseLine;
        switch_index = index_1;
        return baseLine;
    }
    else
    {
        index_1 = switch_index;
        if(switch_index == subPolygon.points.size()-1)
        {
            index_2 = 0;
        }
        else
        {
            index_2 = switch_index + 1;
        }
        Eigen::Vector2f baseVector(subPolygon.points[index_2].x - subPolygon.points[index_1].x,
                                subPolygon.points[index_2].y - subPolygon.points[index_1].y);
        Line baseLine(baseVector, subPolygon.points[index_1], index_1, index_2);
        baseLine_ = baseLine;
        return baseLine;
    }
    
    
}

Line coverage::generateBaseLineFromStartPose(geometry_msgs::PoseStamped start, double theta, bool useStartHeading)
{
    // double vectorAngle = 0;
    // if (useStartHeading)
    // {
    //     vectorAngle = quaterniond2Euler(start);
    // }
    // else
    // {
    //     vectorAngle = theta;
    // }
    // geometry_msgs::Point32 startPoint;
    // startPoint.x = start.pose.position.x;
    // startPoint.y = start.pose.position.y;
    // Line baseLine(Eigen::Vector2f(tan(vectorAngle), 1), startPoint);
    // baseLine_ = baseLine;
    // return baseLine;
}

std::vector<Segment> coverage::generateSegmentsFromPolygon(geometry_msgs::Polygon polygon)
{
    Segment sBuf;
    std::vector<Segment> polygonSegments;
    for (size_t i = 0; i < polygon.points.size() - 1; i++)
    {
        sBuf.endpointA = polygon.points[i];
        sBuf.endpointB = polygon.points[i + 1];
        polygonSegments.push_back(sBuf);
    }
    sBuf.endpointA = polygon.points[polygon.points.size() - 1];
    sBuf.endpointB = polygon.points[0];
    polygonSegments.push_back(sBuf);
    return polygonSegments;
}

double coverage::getSignedDistanceOfPoint2Line(geometry_msgs::Point32 vertex, Line baseLine)
{
    Eigen::Vector2f leftNormal(-baseLine.vector[1], baseLine.vector[0]);
    leftNormal.normalize();
    Eigen::Vector2f vectorAP(vertex.x - baseLine.point.x, vertex.y - baseLine.point.y);
    double signedDistance = vectorAP.dot(leftNormal);
    return signedDistance;
}

int coverage::calculatePolygonDirection(geometry_msgs::Polygon subPolygon, Line baseLine)
{
    double d = 0;
    for (size_t i = 0; i < subPolygon.points.size(); i++)
    {
        if ((i != baseLine.index_1) && (i != baseLine.index_2))
        {
            d = getSignedDistanceOfPoint2Line(subPolygon.points[i], baseLine);
            if (d > 0)
            {
                return 1;
            }
        }
    }
    return 2;
}

geometry_msgs::Polygon coverage::calculateSubPolygon(geometry_msgs::Polygon polygon)
{
    geometry_msgs::Polygon subPolygon;
    geometry_msgs::Point32 vertexBuf;
    vertexBuf = calculateVertexOfSubPolygon(polygon.points[0], polygon.points[polygon.points.size() - 1],
                                            polygon.points[1]);
    subPolygon.points.push_back(vertexBuf);
    for (size_t i = 1; i < polygon.points.size() - 1; i++)
    {
        vertexBuf = calculateVertexOfSubPolygon(polygon.points[i], polygon.points[i - 1], polygon.points[i + 1]);
        subPolygon.points.push_back(vertexBuf);
    }
    vertexBuf = calculateVertexOfSubPolygon(polygon.points[polygon.points.size() - 1], polygon.points[polygon.points.size() - 2], polygon.points[0]);
    subPolygon.points.push_back(vertexBuf);
    return subPolygon;
}

geometry_msgs::Point32 coverage::calculateVertexOfSubPolygon(geometry_msgs::Point32 vertex, geometry_msgs::Point32 neighbor1,
                                                             geometry_msgs::Point32 neighbor2)
{
    geometry_msgs::Point32 subVertex;
    Eigen::Vector2f v1, v2, vb;
    double angle;
    v1[0] = neighbor1.x - vertex.x;
    v1[1] = neighbor1.y - vertex.y;
    v2[0] = neighbor2.x - vertex.x;
    v2[1] = neighbor2.y - vertex.y;
    v1.normalize();
    v2.normalize();
    vb = v1 + v2;
    vb.normalize();
    angle = calculateAngleOfTwoVector(vb, v1);
    double b = zoomDistance_ / sin(angle);
    subVertex.x = b * vb[0] + vertex.x;
    subVertex.y = b * vb[1] + vertex.y;
    return subVertex;
}
double coverage::calculateAngleOfTwoVector(Eigen::Vector2f v1, Eigen::Vector2f v2)
{
    double angle = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
    return angle;
}