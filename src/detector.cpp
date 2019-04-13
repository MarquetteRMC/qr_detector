/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/


#include <ros/ros.h>
#include <nodelet/loader.h>

 int main(int argc, char **argv){
	ros::init(argc, argv, "barcode_reader_node");
	
	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "qr_detector/qr_detector_nodelet" , remap, nargv); //testing on the V2 version of the nodelet. Have the node publish the qr_code data to a new node which then deals with the cmd_vel start. Trying to feed in IMU data for changing the cmd_vel for start up spinning
	ros::spin();
	
	return 0;
}



