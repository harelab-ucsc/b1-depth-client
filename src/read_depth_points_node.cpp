#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <iostream>
#include <sstream>
#include <stdlib.h>

#include <zmq.hpp>

#include "msg_helpers.h"

#define DATA_SIZE 6144000
#define HEIGHT 480
#define WIDTH 640

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "read_depth_points_node");
    
    ros::NodeHandle node;
    ros::Rate loop_rate(100);

    ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud2>("/nx1_camera_forward/depth/color/points", 1);

    zmq::context_t ctx;

    std::string ipAddress = "192.168.123.23:5556";

    std::cout << "Start listening on " << ipAddress << "...\n" << std::endl;
    zmq::socket_t subscriber (ctx, zmq::socket_type::sub);
    subscriber.connect("tcp://" + ipAddress);

    subscriber.set(zmq::sockopt::subscribe, "");

    while (ros::ok()) {
        zmq::message_t messageInfo;
        zmq::message_t messagePoints;
        subscriber.recv(messageInfo, zmq::recv_flags::none);
        subscriber.recv(messagePoints, zmq::recv_flags::none);

        PointCloudInfo info = deserialize(messageInfo.data<uint8_t>(), messageInfo.size());
        
        std::cout << "Got message with ts: " << info.timestamp << std::endl;

        float* points = messagePoints.data<float>();

        // point cloud message code from here: https://medium.com/@tonyjacob_/pointcloud2-message-explained-853bd9907743
        sensor_msgs::PointCloud2 pcl_msg;
            
        //Modifier to describe what the fields are.
        sensor_msgs::PointCloud2Modifier modifier(pcl_msg);

        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32);

        //Msg header
        pcl_msg.header = std_msgs::Header();
        pcl_msg.header.stamp = ros::Time::now();
        if (info.index == 0) {
            pcl_msg.header.frame_id = "nx1_camera_forward_link";
        } else {
            pcl_msg.header.frame_id = "nx1_camera_down_link";
        }

        pcl_msg.height = HEIGHT;
        pcl_msg.width = WIDTH;
        pcl_msg.is_dense = true;

        //Total number of bytes per point
        pcl_msg.point_step = 12;
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width * pcl_msg.height;
        pcl_msg.data.resize(pcl_msg.row_step);

        //Iterators for PointCloud msg
        sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");

        for (int i = 0; i < DATA_SIZE / 3; i++) {  
            *iterZ = points[i*3];
            *iterY = points[i*3+1];
            *iterX = points[i*3+2];

            // Increment the iterators
            ++iterX;
            ++iterY;
            ++iterZ;
        }

        pcl_pub.publish(pcl_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}