#include "Turning.h"
#include <iostream>
#include "tf/transform_datatypes.h" //转换函数头文件
#include "MathCommon.h"
#include "nav_msgs/Odometry.h"
#include "comm.hpp"

#define PI 3.1415926535898
using namespace std;

CarModel::CarModel(): vMax(3), wMax(0.3),plowWidth(3)
{
    // turnRadiusMin(8), carLength(8), carWidth(6), plowWidth(3)
    turnRadiusMin = getParam<double>("~turnRadius",8);
    carLength = getParam<double>("~carLength",9);
    carWidth = getParam<double>("~carWidth",6);
    ROS_INFO("carMOde info:%f,%f,%f",turnRadiusMin,carLength,carWidth);
}
Turning::Turning()
{
}

nav_msgs::Path Turning::generateFinalPath(nav_msgs::Path originalPath, geometry_msgs::PoseStamped current_pose)
{
    
    nav_msgs::Path oriPath = originalPath;
    nav_msgs::Path turning;
    nav_msgs::Path final;
    geometry_msgs::PoseStamped start, end;
    addOrientationToEveryPoint(oriPath);
    final.header.frame_id = "map";
    final.poses.push_back(current_pose);
    final.poses.push_back(oriPath.poses[0]);
    assert(oriPath.poses.size() % 2 == 0);

    if (oriPath.poses.size() % 2 == 0)
    {
        for (size_t i = 1; i < oriPath.poses.size() - 1; i = i + 2)
        {
            start = oriPath.poses[i];
            end = oriPath.poses[i + 1];
            turning = calculateTurningPath(start, end, carModel_);
            addTwoPath(final, turning);
            final.poses.push_back(oriPath.poses[i + 2]);
            interpolateToPath2(final , 2 , oriPath.poses[i + 2]);
        }
        final.poses.push_back(oriPath.poses[oriPath.poses.size()-1]);
        //interpolateToPath(final, 0.5);
        //interpolateToPath(final, 0.5);
    }
    else
    {
        ROS_ERROR("Can't generate a new path!");
        final.poses.clear();
        return final;
    }
    return final;
   
  /*  nav_msgs::Path oriPath = originalPath;
    nav_msgs::Path turning;
    nav_msgs::Path final;
    geometry_msgs::PoseStamped front, start, end;
    addOrientationToEveryPoint(oriPath);
    final.header.frame_id = "map";
    final.poses.push_back(current_pose);
    final.poses.push_back(oriPath.poses[0]);
    interpolateToPath(final, 1);
    assert(oriPath.poses.size() % 2 == 0);

    if (oriPath.poses.size() % 2 == 0)
    {
        for (size_t i = 1; i < oriPath.poses.size() - 1; i = i + 2)
        {  
            front = oriPath.poses[i-1];
            start = oriPath.poses[i];
            end = oriPath.poses[i + 1];
            nav_msgs::Path tmp_path;
            tmp_path.poses.clear();
            tmp_path.poses.push_back(front);
            tmp_path.poses.push_back(start);
            interpolateToPath(tmp_path, 0.5);
            turning = calculateTurningPath(start, end, carModel_);
            addTwoPath(tmp_path, turning);
            addTwoPath(final, tmp_path);
        }
        
        interpolateToPath(final, 0.5);
    }
    else
    {
        ROS_ERROR("Can't generate a new path!");
        final.poses.clear();
        return final;
    }
    return final; */
}

nav_msgs::Path Turning::calculateTurningPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car)
{
    double eex = 0;
    double eey = 0;
    double ctheta;
    nav_msgs::Path turning_path;
    turning_path.header.frame_id = "map";
    ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
    eex = end.pose.position.x;
    eey = end.pose.position.y;
    transformFromMap2Car(eex, eey, start.pose.position.x, start.pose.position.y, ctheta);
    cout << "start = " << start.pose.position.x << "/" << start.pose.position.y << endl;
    cout << "end = " << end.pose.position.x << "/" << end.pose.position.y << endl;
    if (fabs(eex) < carModel_.turnRadiusMin)
    {
        turning_path = calculateFishTailPath(start, end, car);
        cout << "fish tail = " << eex << endl;
        cout << "--------------------------------" << endl;
    }

    else if (fabs(eex) < 2 * carModel_.turnRadiusMin)
    {
        turning_path = calculateBulbPath(start, end, car);
        cout << "bulb = " << eex << endl;
        cout << "--------------------------------" << endl;
    }
    else if (fabs(eex) >= 2 * carModel_.turnRadiusMin)
    {
        turning_path = calculateBowPath(start, end, car);
        cout << "bow = " << eex << endl;
        cout << "--------------------------------" << endl;
    }
    return turning_path;
}

void Turning::addOrientationToEveryPoint(nav_msgs::Path &originpath)
{
    double theta = getAngle(originpath.poses[0], originpath.poses[1]);
    originpath.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    for (size_t i = 1; i < originpath.poses.size(); i++) //方向为当前点-前一点
    {
        theta = getAngle(originpath.poses[i - 1], originpath.poses[i]);
        originpath.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    }
}

nav_msgs::Path Turning::calculateFishTailPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car)
{
    nav_msgs::Path fish_tail;
    fish_tail.header.frame_id = "map";
    geometry_msgs::PoseStamped p;
    geometry_msgs::PoseStamped pBuf;
    geometry_msgs::PoseStamped first;
    geometry_msgs::PoseStamped second;
    // std::vector<geometry_msgs::PoseStamped> fish_tail_.poses;
    double x = 0;
    double y = 0;
    double fx, fy, sx, sy, ex, ey;
    double ptheta = 0;
    double keyangle = 0;
    double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
    ex = end.pose.position.x;
    ey = end.pose.position.y;
    transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);
    p.pose.position.z = 0;

    if (ex < 0)
    {
        p = end;
        end = start;
        end.pose.orientation = p.pose.orientation;
        start.pose.position = p.pose.position;
        double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
        ex = end.pose.position.x;
        ey = end.pose.position.y;
        transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);
        keyangle = atan2(carModel_.turnRadiusMin, (ey * carModel_.turnRadiusMin) / (2 * carModel_.turnRadiusMin - ex));
        fx = carModel_.turnRadiusMin + carModel_.turnRadiusMin * cos(keyangle);
        fy = carModel_.turnRadiusMin * sin(keyangle);
        sx = -1 * carModel_.turnRadiusMin + ex + carModel_.turnRadiusMin * cos(keyangle);
        sy = carModel_.turnRadiusMin * sin(keyangle) + ey;
        cout << "fx/fy = " << fx << "/" << fy << endl;
        cout << "sx/sy = " << sx << "/" << sy << endl;
        cout << "ctheta = " << ctheta << endl;
        pBuf = start;
        p.pose.position.z = 0;
        //start不进,first进
        for (size_t fi = 1; fi < 1001; fi++)
        {
            x = (fx / 1000) * fi;
            y = sqrt((2 * carModel_.turnRadiusMin - x) * x);
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.001;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            fish_tail.poses.push_back(p);
            pBuf = p;
        }
        second = p;
        //first不进,second进
        for (size_t si = 1; si < 1001; si++)
        {
            x = fx + ((sx - fx) / 1000) * si;
            y = ((x - fx) * (sy - fy) / (sx - fx)) + fy;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.002;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            fish_tail.poses.push_back(p);
        }
        first = p;
        pBuf = second;
        //second不进,end进
        for (size_t ei = 1; ei < 1001; ei++)
        {

            x = sx + ((ex - sx) / 1000) * ei;
            y = sqrt((2 * carModel_.turnRadiusMin + x - ex) * (ex - x)) + ey;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.001;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            fish_tail.poses.push_back(p);
            pBuf = p;
        }
        // fish_tail_.poses.reserve(fish_tail_.poses.size());
        std::reverse(fish_tail.poses.begin(), fish_tail.poses.end());
    }
    else if (ex > 0)
    {
        keyangle = atan2(carModel_.turnRadiusMin, (ey * carModel_.turnRadiusMin) / (2 * carModel_.turnRadiusMin - ex));
        fx = carModel_.turnRadiusMin + carModel_.turnRadiusMin * cos(keyangle);
        fy = carModel_.turnRadiusMin * sin(keyangle);
        sx = -1 * carModel_.turnRadiusMin + ex + carModel_.turnRadiusMin * cos(keyangle);
        sy = carModel_.turnRadiusMin * sin(keyangle) + ey;
        cout << "fx/fy = " << fx << "/" << fy << endl;
        cout << "sx/sy = " << sx << "/" << sy << endl;
        cout << "ctheta = " << ctheta << endl;
        pBuf = start;
        //start不进,first进
        for (size_t fi = 1; fi < 1001; fi++)
        {
            x = (fx / 1000) * fi;
            y = sqrt((2 * carModel_.turnRadiusMin - x) * x);
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.001;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            // }
            fish_tail.poses.push_back(p);
            pBuf = p;
        }
        first = p;
        //first不进,second进
        for (size_t si = 1; si < 1001; si++)
        {
            x = fx + ((sx - fx) / 1000) * si;
            y = ((x - fx) * (sy - fy) / (sx - fx)) + fy;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.002;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            fish_tail.poses.push_back(p);
        }

        second = p;
        pBuf = second;
        //second不进,end进
        for (size_t ei = 1; ei < 1001; ei++)
        {
            x = sx + ((ex - sx) / 1000) * ei;
            y = sqrt((2 * carModel_.turnRadiusMin + x - ex) * (ex - x)) + ey;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.001;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            fish_tail.poses.push_back(p);
            pBuf = p;
        }
    }
    else
    {
        ROS_ERROR("Failed to calculate key points, direction solution error");
        return fish_tail;
    }

    return fish_tail;
}

nav_msgs::Path Turning::calculateBulbPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car)
{
    nav_msgs::Path bulb_path;
    geometry_msgs::PoseStamped p;
    geometry_msgs::PoseStamped pBuf;
    geometry_msgs::PoseStamped first;
    geometry_msgs::PoseStamped second;
    // std::vector<geometry_msgs::PoseStamped> fish_tail_.poses;
    bulb_path.header.frame_id = "map";
    double r = car.turnRadiusMin;
    double x = 0;
    double y = 0;
    double fx, fy, sx, sy, ex, ey, ox, oy, x1, x2;
    double ptheta = 0;
    double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
    ex = end.pose.position.x;
    ey = end.pose.position.y;
    transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);
    p.pose.position.z = 0;
    if (ex < 0)
    {
        p = end;
        end = start;
        end.pose.orientation = p.pose.orientation;
        start.pose.position = p.pose.position;
        const double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
        ex = end.pose.position.x;
        ey = end.pose.position.y;
        transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);
        intersectionOf2Circles(-r, 0, 2 * r, (r + ex), ey, 2 * r, ox, oy);            //计算ox,oy
        intersectionOf2Circles(-r, 0, r + 0.0002, ox, oy, r + 0.0002, fx, fy);        //算fx,fy
        intersectionOf2Circles((r + ex), ey, r + 0.0002, ox, oy, r + 0.0002, sx, sy); //算sx,sy

        x1 = ox - r + 0.0002;
        x2 = ox + r - 0.0002;
        pBuf = start;
        p.pose.position.z = 0;

        //start不进,first进
        for (size_t fi = 1; fi < 1001; fi++)
        {
            x = (fx / 1000) * fi;
            y = sqrt(-x * (2 * r + x));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
        first = p;

        //first不进,second进
        for (size_t i1 = 1; i1 < 251; i1++) //first-(x1,y1)
        {
            x = fx + ((x1 - fx) / 250) * i1;
            y = oy - sqrt((r + x - ox) * (r - x + ox));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
        for (size_t i2 = 0; i2 < 501; i2++)
        {
            x = x1 + ((x2 - x1) / 500) * i2;
            y = sqrt((r + x - ox) * (r - x + ox)) + oy;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
        for (size_t i3 = 0; i3 < 251; i3++)
        {
            x = x2 + ((sx - x2) / 250) * i3;
            y = oy - sqrt((r + x - ox) * (r - x + ox));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }

        second = p;
        pBuf = second;
        //second不进,end进
        for (size_t ei = 1; ei < 1001; ei++)
        {
            x = sx + ((ex - sx) / 1000) * ei;
            y = sqrt((x - ex) * (2 * r + ex - x)) + ey;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
        // fish_tail_.poses.reserve(fish_tail_.poses.size());
        std::reverse(bulb_path.poses.begin(), bulb_path.poses.end());
    }
    else if (ex > 0)
    {
        bulb_path.poses.push_back(start);
        const double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
        ex = end.pose.position.x;
        ey = end.pose.position.y;
        transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);
        intersectionOf2Circles(-r, 0, 2 * r, (r + ex), ey, 2 * r, ox, oy);            //计算ox,oy
        intersectionOf2Circles(-r, 0, r + 0.0002, ox, oy, r + 0.0002, fx, fy);        //算fx,fy
        intersectionOf2Circles((r + ex), ey, r + 0.0002, ox, oy, r + 0.0002, sx, sy); //算sx,sy
        x1 = ox - r + 0.0002;
        x2 = ox + r - 0.0002;
        pBuf = start;
        //start不进,first进
        for (size_t fi = 1; fi < 1001; fi++)
        {
            x = (fx / 1000) * fi;
            y = sqrt(-x * (2 * r + x));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            // }
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
        first = p;

        //first不进,second进
        for (size_t i1 = 1; i1 < 251; i1++) //first-(x1,y1)
        {
            x = fx + ((x1 - fx) / 250) * i1;
            y = oy - sqrt((r + x - ox) * (r - x + ox));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
        for (size_t i2 = 0; i2 < 501; i2++)
        {
            x = x1 + ((x2 - x1) / 500) * i2;
            y = sqrt((r + x - ox) * (r - x + ox)) + oy;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
        for (size_t i3 = 0; i3 < 251; i3++)
        {
            x = x2 + ((sx - x2) / 250) * i3;
            y = oy - sqrt((r + x - ox) * (r - x + ox));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }

        second = p;
        pBuf = second;
        //second不进,end进
        for (size_t ei = 1; ei < 1001; ei++)
        {
            x = sx + ((ex - sx) / 1000) * ei;
            y = sqrt((x - ex) * (2 * r + ex - x)) + ey;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bulb_path.poses.push_back(p);
            pBuf = p;
        }
    }
    else
    {
        ROS_ERROR("Failed to calculate key points, direction solution error");
    }
    cout << "path_size = " << bulb_path.poses.size() << endl;
    return bulb_path;
}

nav_msgs::Path Turning::calculateBowPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, const CarModel car)
{
    nav_msgs::Path bow_path;
    geometry_msgs::PoseStamped p;
    geometry_msgs::PoseStamped pBuf;
    geometry_msgs::PoseStamped first;
    geometry_msgs::PoseStamped second;
    bow_path.header.frame_id = "map";
    // std::vector<geometry_msgs::PoseStamped> fish_tail_.poses;
    double r = car.turnRadiusMin;
    double x = 0;
    double y = 0;
    double fx, fy, sx, sy, ex, ey, x1, x2;
    double ptheta = 0;
    double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
    ex = end.pose.position.x;
    ey = end.pose.position.y;
    transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);

    if (ex < 0)
    {
        const double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
        p = end;
        end = start;
        end.pose.orientation = p.pose.orientation;
        start.pose.position = p.pose.position;
        // bow_path_.poses.push_back(start);
        // const double ctheta = addTwoAngle(Quaterniond2Euler(start),-PI/2);
        ex = end.pose.position.x;
        ey = end.pose.position.y;
        transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);
        double k = pow((2 * r - ex) / ey, 2) + 1;
        if (ey >= 0)
        {
            fx = r * (k - sqrt(k)) / k;
        }
        else
        {
            fx = r * (k + sqrt(k)) / k;
        }

        fy = ((2 * r - ex) / ey) * (fx - r);
        sx = ex + fx - 2 * r;
        sy = ey + fy;
        pBuf = start;
        p.pose.position.z = 0;
        //start不进,first进
        for (size_t fi = 1; fi < 1001; fi++)
        {
            x = (fx / 1000) * fi;
            y = sqrt(x * (2 * r - x));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bow_path.poses.push_back(p);
            pBuf = p;
        }
        first = p;

        // first不进,second进
        for (size_t si = 1; si < 1001; si++)
        {
            x = fx + ((sx - fx) / 1000) * si;
            y = ((x - fx) * (sy - fy) / (sx - fx)) + fy;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bow_path.poses.push_back(p);
            pBuf = p;
        }

        second = p;
        pBuf = second;
        //second不进,end进
        for (size_t ei = 1; ei < 1001; ei++)
        {
            x = sx + ((ex - sx) / 1000) * ei;
            y = sqrt((2 * r + x - ex) * (ex - x)) + ey;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bow_path.poses.push_back(p);
            pBuf = p;
        }

        std::reverse(bow_path.poses.begin(), bow_path.poses.end());
    }
    else if (ex > 0)
    {
        bow_path.poses.push_back(start);
        const double ctheta = addTwoAngle(quaterniond2Euler(start), -PI / 2);
        ex = end.pose.position.x;
        ey = end.pose.position.y;
        transformFromMap2Car(ex, ey, start.pose.position.x, start.pose.position.y, ctheta);
        double k = pow((2 * r - ex) / ey, 2) + 1;
        if (ey >= 0)
        {
            fx = r * (k - sqrt(k)) / k;
        }
        else
        {
            fx = r * (k + sqrt(k)) / k;
        }
        fy = ((2 * r - ex) / ey) * (fx - r);
        sx = ex + fx - 2 * r;
        sy = ey + fy;
        pBuf = start;
        //start不进,first进
        for (size_t fi = 1; fi < 1001; fi++)
        {
            x = (fx / 1000) * fi;
            y = sqrt(x * (2 * r - x));
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            // }
            bow_path.poses.push_back(p);
            pBuf = p;
        }
        first = p;

        // first不进,second进
        for (size_t si = 1; si < 1001; si++)
        {
            x = fx + ((sx - fx) / 1000) * si;
            y = ((x - fx) * (sy - fy) / (sx - fx)) + fy;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bow_path.poses.push_back(p);
        }

        second = p;
        pBuf = second;
        //second不进,end进
        for (size_t ei = 1; ei < 1001; ei++)
        {
            x = sx + ((ex - sx) / 1000) * ei;
            y = sqrt((2 * r + x - ex) * (ex - x)) + ey;
            transformFromCar2Map(x, y, start.pose.position.x, start.pose.position.y, ctheta);
            p.header.frame_id = "map";
            p.pose.position.x = x;
            p.pose.position.y = y;
            ptheta = getAngle(pBuf, p);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            bow_path.poses.push_back(p);
            pBuf = p;
        }
    }
    else
    {
        ROS_ERROR("Failed to calculate key points, direction solution error");
    }
    cout << "path_size = " << bow_path.poses.size() << endl;
    return bow_path;
}
