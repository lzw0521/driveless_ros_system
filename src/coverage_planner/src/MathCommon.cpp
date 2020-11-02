#include <cmath>
#include "MathCommon.h"
#define PI 3.1415926535898

// 有两套笛卡尔坐标系：XOY和X'O'Y'，X'O'Y'相对于XOY坐标系进行了平移(x_shift, y_shift)和旋转（angle，弧度）变换，
// 一个点在XOY坐标系中的坐标为：(x, y)，计算该点在X'O'Y'坐标系中的坐标：(x_prime, y_prime)。
// x = x_prime * cos(angle) - y_prime * sin(angle) + x_shift
// y = y_prime * cos(angle) + x_prime * sin(angle) + y_shift
// x_prime = (x - x_shift) * cos(angle) + (y - y_shift) * sin(angle)
// y_prime = (y - y_shift) * cos(angle) - (x - x_shift) * sin(angle)
void transformFromCar2Map(double &x, double &y, double a, double b, double theta)
{
    double xo = x;
    double yo = y;

    x = xo * cos(theta) - yo * sin(theta) + a;
    y = yo * cos(theta) + xo * sin(theta) + b;
}

void transformFromMap2Car(double &x, double &y, double a, double b, double theta)
{
    double xo = x;
    double yo = y;
    x = (xo - a) * cos(theta) + (yo - b) * sin(theta);
    y = (yo - b) * cos(theta) - (xo - a) * sin(theta);
}
double addTwoAngle(double a, double b)
{
    // return ((a + b) < PI) ? (a + b) : (a + b - PI);
    double c = a + b;
    if (c > PI)
    {
        c = c - 2 * PI;
    }
    else if (c < -PI)
    {
        c = c + 2 * PI;
    }

    return c;
}

double getDistance(double poseCur[2], double lookahead[2])
{

    return sqrt(pow((poseCur[0] - lookahead[0]), 2) + pow((poseCur[1] - lookahead[1]), 2));
}

double getDistance(geometry_msgs::PoseStamped poseCur, geometry_msgs::PoseStamped lookahead)
{
    double start[2] = {0};
    double des[2] = {0};
    start[0] = poseCur.pose.position.x;
    start[1] = poseCur.pose.position.y;
    des[0] = lookahead.pose.position.x;
    des[1] = lookahead.pose.position.y;
    return getDistance(start, des);
}

double getAngle(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
    return (atan2(b.pose.position.y - a.pose.position.y, b.pose.position.x - a.pose.position.x));
}

double angleDiff(double a, double b)
{
    double d1, d2;
    d1 = a - b;
    if (d1 > PI)
    {
        d1 = d1 - 2 * PI;
        // d1 = 2 * PI - d1;
    }
    else if (d1 < -PI)
    {
        d1 = 2 * PI + d1;
    }
    return d1;
}
double quaterniond2Euler(geometry_msgs::PoseStamped data)
{
    double x = data.pose.orientation.x;
    double y = data.pose.orientation.y;
    double z = data.pose.orientation.z;
    double w = data.pose.orientation.w;
    double theta = 0;
    double euler[3] = {0};
    threeAxisRot(-2 * (y * z - w * x),
                 w * w - x * x - y * y + z * z,
                 2 * (x * z + w * y),
                 -2 * (x * y - w * z),
                 w * w + x * x - y * y - z * z,
                 euler);
    theta = euler[0];
    // std::cout << "euler0=" << euler[0] << "\teuler1=" << euler[1] << "\teuler2=" << euler[2] << std::endl;
    return (theta);
}

void threeAxisRot(double r11, double r12, double r21, double r31, double r32, double *euler)
{
    euler[0] = atan2(r31, r32);
    euler[1] = asin(r21);
    euler[2] = atan2(r11, r12);
}

void intersectionOf2Circles(double a1, double b1, double R1, double a2, double b2, double R2, double &x, double &y)
{
    double x1 = 0;
    double x2 = 0;
    double y1 = 0;
    double y2 = 0;

    double R1R1 = R1 * R1;
    double a1a1 = a1 * a1;
    double b1b1 = b1 * b1;

    double a2a2 = a2 * a2;
    double b2b2 = b2 * b2;
    double R2R2 = R2 * R2;

    double subs1 = a1a1 - 2 * a1 * a2 + a2a2 + b1b1 - 2 * b1 * b2 + b2b2;
    double subs2 = -R1R1 * a1 + R1R1 * a2 + R2R2 * a1 - R2R2 * a2 + a1a1 * a1 - a1a1 * a2 - a1 * a2a2 + a1 * b1b1 - 2 * a1 * b1 * b2 + a1 * b2b2 + a2a2 * a2 + a2 * b1b1 - 2 * a2 * b1 * b2 + a2 * b2b2;
    double subs3 = -R1R1 * b1 + R1R1 * b2 + R2R2 * b1 - R2R2 * b2 + a1a1 * b1 + a1a1 * b2 - 2 * a1 * a2 * b1 - 2 * a1 * a2 * b2 + a2a2 * b1 + a2a2 * b2 + b1b1 * b1 - b1b1 * b2 - b1 * b2b2 + b2b2 * b2;
    double sigma = sqrt((R1R1 + 2 * R1 * R2 + R2R2 - a1a1 + 2 * a1 * a2 - a2a2 - b1b1 + 2 * b1 * b2 - b2b2) * (-R1R1 + 2 * R1 * R2 - R2R2 + subs1));
    if (abs(subs1) > 0.0000001) //分母不为0,已求两点
    {
        x1 = (subs2 - sigma * b1 + sigma * b2) / (2 * subs1);
        x2 = (subs2 + sigma * b1 - sigma * b2) / (2 * subs1);

        y1 = (subs3 + sigma * a1 - sigma * a2) / (2 * subs1);
        y2 = (subs3 - sigma * a1 + sigma * a2) / (2 * subs1);
    }
    if (y1 > y2) //输出y大的一点
    {
        x = x1;
        y = y1;
    }
    else
    {
        x = x2;
        y = y2;
    }
}
void interpolateToPath2(nav_msgs::Path &path, double interval,geometry_msgs::PoseStamped _end)
{
    nav_msgs::Path pathBuf = path;
    geometry_msgs::PoseStamped pBuf;
    double ptheta;
    //path.poses.clear();

    pBuf = pathBuf.poses[pathBuf.poses.size() - 1];

    geometry_msgs::PoseStamped start, end;
    if (pathBuf.poses.empty())
    {
        return;
    }
   
        start = pathBuf.poses[pathBuf.poses.size() - 1];
        end = _end;
        double dx, dy;
        double dist = getDistance(start, end);
        int val = floor(dist / interval);
        if (val != 0)
        {
            dx = (end.pose.position.x - start.pose.position.x) / val;
            dy = (end.pose.position.y - start.pose.position.y) / val;
        }
        else
        {
            dx = 0;
            dy = 0;
        }

        //start进,end不进
        for (int i = 0; i <= val; i++)
        {
            geometry_msgs::PoseStamped p;
            p.header.frame_id = "map";
            p.pose.position.x = start.pose.position.x + dx * i;
            p.pose.position.y = start.pose.position.y + dy * i;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            p.header.seq = 0;
            path.poses.push_back(p);
            pBuf = p;
        }
        path.poses.push_back(_end);

}
void interpolateToPath(nav_msgs::Path &path, double interval)
{
    nav_msgs::Path pathBuf = path;
    geometry_msgs::PoseStamped pBuf;
    double ptheta;
    //path.poses.clear();

    pBuf = pathBuf.poses[0];

    geometry_msgs::PoseStamped start, end;
    if (pathBuf.poses.empty())
    {
        return;
    }
    for (size_t index = 0; index < pathBuf.poses.size() - 1; index++)
    {
        start = pathBuf.poses[index];
        end = pathBuf.poses[index + 1];
        double dx, dy;
        double dist = getDistance(start, end);
        int val = floor(dist / interval);
        if (val != 0)
        {
            dx = (end.pose.position.x - start.pose.position.x) / val;
            dy = (end.pose.position.y - start.pose.position.y) / val;
        }
        else
        {
            dx = 0;
            dy = 0;
        }

        //start进,end不进
        for (int i = 0; i <= val; i++)
        {
            geometry_msgs::PoseStamped p;
            p.header.frame_id = "map";
            p.pose.position.x = start.pose.position.x + dx * i;
            p.pose.position.y = start.pose.position.y + dy * i;
            ptheta = getAngle(p, pBuf);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(ptheta);
            p.header.seq = 0;
            path.poses.push_back(p);
            pBuf = p;
        }
    }
}

void addTwoPath(nav_msgs::Path &final, nav_msgs::Path turn)
{
    for (size_t i = 0; i < turn.poses.size(); i++)
    {
        final.poses.push_back(turn.poses[i]);
    }
}
