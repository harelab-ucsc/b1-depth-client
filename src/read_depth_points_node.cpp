#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <iostream>
#include <sstream>
#include <stdlib.h>

#include <zmq.hpp>

#include "msg_helpers.h"

#define HEIGHT 480
#define WIDTH 640

#define NUM_CAMS 5

const std::string cameras[NUM_CAMS] = {
    "nx1_camera_down",
    "nx1_camera_forward",
    "nx2_camera_left",
    "nx2_camera_right",
    "nx3_camera_backward"
};

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "read_depth_points_node");
    ROS_INFO("Started read_depth_points node\n");
    
    ros::NodeHandle node;
    ros::Rate loop_rate(1000);

    ros::Publisher pcl_pubs[NUM_CAMS];
    for (uint8_t i = 0; i < NUM_CAMS; i++) {
        ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud2>("/" + cameras[i] + "/depth/color/points", 1);
        pcl_pubs[i] = pcl_pub;
    }

    zmq::context_t ctx;


    ROS_INFO("Start listening for pointcloud data...\n");
    zmq::socket_t subscriber (ctx, zmq::socket_type::sub);
    // each of the NX ports
    subscriber.connect("tcp://192.168.123.23:5556");
    subscriber.connect("tcp://192.168.123.24:5556");
    subscriber.connect("tcp://192.168.123.25:5556");

    subscriber.set(zmq::sockopt::subscribe, "");

    while (ros::ok()) {
        zmq::message_t messageInfo;
        subscriber.recv(messageInfo, zmq::recv_flags::none);

        PointCloudInfo info = deserialize(messageInfo.data<uint8_t>(), messageInfo.size());
        
        zmq::message_t messagePoints;
        subscriber.recv(messagePoints, zmq::recv_flags::none);

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
        pcl_msg.header.stamp = ros::Time(info.timestamp / 1000.0);
        pcl_msg.header.frame_id = cameras[info.index] + "_link";

        pcl_msg.height = HEIGHT;
        pcl_msg.width = WIDTH;
        pcl_msg.is_dense = true;

        //Total number of bytes per point
        pcl_msg.point_step = 12;
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
        pcl_msg.data.resize(pcl_msg.row_step * pcl_msg.height);
        if (pcl_msg.row_step * pcl_msg.height != messagePoints.size()) {
            ROS_ERROR("Message points size does not match expected! Expected: %d, got %ld", pcl_msg.row_step * pcl_msg.height, messagePoints.size());
        }

        //Iterators for PointCloud msg
        sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");

        for (int i = 0; i <= (messagePoints.size() / sizeof(float)) - 3; i += 3) {  
            *iterZ = points[i];
            *iterY = points[i+1];
            *iterX = points[i+2];

            // Increment the iterators
            ++iterX;
            ++iterY;
            ++iterZ;
        }

        pcl_pubs[info.index].publish(pcl_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}