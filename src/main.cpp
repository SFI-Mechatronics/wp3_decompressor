#include <time.h>
#include <stdio.h>
#include <signal.h>

#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "wp3_decompressor/decompressor.h"

// Setup
#define MIN_POINTS 3 //minimum number of points in compressed voxels
#define ROS_RATE 60

void killHandler(int)
{
        ROS_INFO("%s","Shutdown request received.");
        ros::NodeHandle nh;
        ROS_INFO("%s","Terminating nodehandle.");
        nh.shutdown();
        ROS_INFO("%s","Terminating rosnode.");
        ros::shutdown();
}

int main(int argc, char **argv)
{
        // Initialize ROS
        ros::init(argc, argv, "wp3_decompressor", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");

        signal(SIGINT, killHandler);
        signal(SIGTERM, killHandler);

        ros::Rate loopRate(ROS_RATE);

        // Multithreaded spinner
//    	ros::AsyncSpinner spinner(8);
//    	spinner.start();
//    	usleep(1000000); // give the spinner some time to start (1000 ms)

        if(!nh.hasParam("sensor_name"))
		    ROS_ERROR("%s","Missing _sensor_name:=<name> parameter! Shutting down...");
        else if(!nh.hasParam("filter_value"))
            ROS_ERROR("%s","Missing _filter_value:=<intensity> parameter! Shutting down...");
        else{
            
            std::string sensorName;
            nh.getParam("sensor_name", sensorName);
            
            int filterValue;
            nh.getParam("filter_value", filterValue);

            ros::Duration(1.0).sleep();

            int sensorType = 0;
            if (nh.hasParam("sensor_type"))
                nh.getParam("sensor_type", sensorType);
            
            std::string outputTopic;
            std::string inputTopic;
            
            switch(sensorType) {
                
                case 0: // Kinect
                            
                    outputTopic = "/master/"+sensorName+"/pc2_decompressed";
                    inputTopic = "/"+sensorName+"/wp3/pc2_compressed";
                    break;
            
                case 1: // Realsense
                            
                    outputTopic = "/master/"+sensorName+"/pc2_decompressed_rs";
                    inputTopic = "/"+sensorName+"/wp3/pc2_compressed_rs";
                    break;
                    
            } // End switch (sensorType)
            
            // Start decompressor
            wp3::CloudDecompressor decompressor(outputTopic, inputTopic, filterValue, false);

            
            while(ros::ok()){
                ros::spinOnce();
                loopRate.sleep();
            }
         }

        if(ros::ok()){
                nh.shutdown();
                ros::shutdown();
        }


        ROS_INFO("%s","Shutdown complete.");
        return 0;
}
