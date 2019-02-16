/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <ros/ros.h>
#include "opencv/cv.h"
#include <nodelet/nodelet.h>
#include "cv_bridge/cv_bridge.h"
#include <string>
#include "boost/unordered_map.hpp"
//#include <image_transport/image_transport.h>

//#include "qr_detector/detector.h"
#include "zbar.h"

namespace qr_detector {

class QrDetectorNodelet : public nodelet::Nodelet
{
public:
    QrDetectorNodelet();
    virtual ~QrDetectorNodelet();

private:
    virtual void onInit();
    void connectCb();
    void disconnectCb();
    void imageCb(const sensor_msgs::ImageConstPtr& image);
    void cleanCb();

   	 ros::NodeHandle nh, private_nh_;
    //image_transport::ImageTransport it;
   // image_transport::Subscriber imgSubscriber;
	ros::Subscriber imgSubscriber = nh.subscribe("image", 10, &QrDetectorNodelet::imageCb, this);   
	ros::Publisher tagsPublisher; //publisher code will need to be hard coded to the ODrive node.
    	zbar::ImageScanner scanner_;
	ros::Timer clean_timer_;
	boost::unordered_map<std::string, ros::Time> barcode_memory_;
    	
	double throttle_;
   // Detector detector;
};

}
