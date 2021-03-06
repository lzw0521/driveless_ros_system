/*
 * @Author: zhujiankang 
 * @Date: 2020-06-6 08:54:55 
 * @Last Modified by:   zhujiankang 
 * @Last Modified time: 2020-06-11 08:54:55 
 */
#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <thread>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>

#include "ub482/serial.h"
#include "ub482/GPS.h"


std::string GPGGA;
const char *serdevice = "/dev/ttyUSB1";
const int baud = 115200; 
int serial_fd;

std::string strPort;

int main(int argc, char** argv) {

    ros::init(argc, argv, "gps_node");
    ros::NodeHandle node = ros::NodeHandle();
    
    //ros::Publisher gpsPublisher = node.advertise<sensor_msgs::NavSatFix>("/sensor/gps", 1);
    ros::Publisher gpsStringPublisher = node.advertise<std_msgs::String>("/satellite_map/gps_current", 1);
    //GPS gps(gpsPublisher);

    //ros::param::get("~Port_GPS",strPort);

    //std::cout << "Port :"<<strPort<<std::endl;

    std_msgs::String gps_pub;
    float i = 0.0;

    //const int maxSize = 20000;
    //char buf[maxSize] = {0};

    // // open serial com
    // if ( ros::ok()) {
    //     serial_fd = uart_open(strPort.c_str());
    //     if (serial_fd) {
    //         int ret = uart_set(serial_fd, baud);
    //         if (!ret) {
    //             fprintf(stderr, "Set serial com failed!\n");
    //             return 1;
    //         }
    //     } else {
    //         fprintf(stderr, "Open serial com failed!\n");
    //         return 1;
    //     }
    // }

    ros::Rate rate(0.1);

    while(ros::ok()) {
        // std::vector<std::string> strVec;
        // memset(buf, 0, maxSize);
        // int i = uart_recv(serial_fd, buf, 100);
        // if (gps.processBuf(i, buf, strVec)) {
        //     for (std::size_t j = 0; j < strVec.size(); j++) {
        //         //std::cout << strVec[j] << std::endl;
        //         gps.processGpsStr(strVec[j]);
        //     }
        // }

       /* gps_pub.data = gps.gps_string;
        gpsStringPublisher.publish(gps_pub);  */
        i = i+0.01;
        ROS_INFO("%f",i);
        gps_pub.data = "{\"lat\":49.3090 ,\"lng\":120.0223  }";
        gpsStringPublisher.publish(gps_pub); 
        ros::spinOnce();
        rate.sleep();
    }
    return 1;
}
