/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "qr_detector/qr_detector_nodelet.h"

#include "pluginlib/class_list_macros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>

//PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);

bool detection = false;

namespace qr_detector {

QrDetectorNodelet::QrDetectorNodelet()
  {
	scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1); 
  }

QrDetectorNodelet::~QrDetectorNodelet()
{
    imgSubscriber.shutdown();
}

void QrDetectorNodelet::onInit()
{
    nh = getNodeHandle();
	private_nh_ = getPrivateNodeHandle();
    
    NODELET_INFO_STREAM("Im in the init stream");
    tagsPublisher = nh.advertise<std_msgs::String>("qr_codes", 10,
                                                   boost::bind(&QrDetectorNodelet::connectCb, this),
                                                   boost::bind(&QrDetectorNodelet::disconnectCb, this));
    vel_Publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	private_nh_.param<double>("throttle_repeated_barcodes", throttle_, 0.0);
	if (throttle_ > 0.0){
		clean_timer_ = nh.createTimer(ros::Duration(10.0), boost::bind(&QrDetectorNodelet::cleanCb, this));
}


    NODELET_INFO_STREAM("Initialising nodelet... [" << nh.getNamespace() << "]");
}

void QrDetectorNodelet::connectCb()
{
	NODELET_INFO_STREAM("I made it into the connectCb method");
    if (!imgSubscriber || tagsPublisher.getNumSubscribers() > 0)
    {
        NODELET_INFO("Connecting to image topic.");
        //bool detection = false;
        imgSubscriber = nh.subscribe("camera/rgb/image_raw", 10, &QrDetectorNodelet::imageCb, this);
    }
}

void QrDetectorNodelet::disconnectCb()
{
    if (tagsPublisher.getNumSubscribers() == 0)
    {
        NODELET_INFO("Unsubscribing from image topic.");
        imgSubscriber.shutdown();
    }
}

void QrDetectorNodelet::imageCb(const sensor_msgs::ImageConstPtr &image)
{

    //bool detection = false;
        
    cv_bridge::CvImageConstPtr cv_image;
	cv_image = cv_bridge::toCvShare(image, "mono8");
	
	
	zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data, cv_image->image.cols * cv_image->image.rows);
	
    geometry_msgs::Twist twist;
    
    if(detection == false){
	twist.linear.x = 0;
	twist.angular.z = -350.0;
	
    vel_Publisher.publish(twist);
    }
	
	scanner_.scan(zbar_image);

	for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
		symbol != zbar_image.symbol_end(); ++symbol)
	{
		std::string barcode = symbol->get_data();
		if(throttle_ > 0.0)
		{
			if (barcode_memory_.count(barcode) > 0)
			{
				if(ros::Time::now() > barcode_memory_.at(barcode))
				{
					NODELET_DEBUG("Memory timed out for barcode, publishing");
					barcode_memory_.erase(barcode);
				}
				else
				{
					continue;
				}
			}
			barcode_memory_.insert(std::make_pair(barcode, ros::Time::now() + ros::Duration(throttle_)));
		}
		//currently to see the data being published run rostopic echo /qr_codes
		//Future work: will need to publish this data to the ODrives node to control the start up spin
		std_msgs::String barcode_string;
		barcode_string.data = barcode;

		ROS_INFO_STREAM("msgs: " << barcode);
		if(barcode == "Left_Code"){
		    //for future reference this will need a countdown timer function
		    //create an if statement to see which QR code is found first, front or back
		    //if front is back needs to turn another 90 degrees, if front is first needs to turn 270 degrees. 
		    //Robot will spin clockwise motion.
		    //once the degree rotation is accomplished will need to publish angular 0 and linear 0.


		    detection = true;		    
		    ROS_INFO_STREAM("did i make it??");
		    twist.linear.x=0;
            twist.angular.z = 0;
            ROS_INFO_STREAM("Did i publish dataaa");
            }
        
        vel_Publisher.publish(twist);
		tagsPublisher.publish(barcode_string);
	}
	
	zbar_image.set_data(NULL, 0);
  }
  void QrDetectorNodelet::cleanCb()
  {
	for(boost::unordered_map<std::string, ros::Time>::iterator it = barcode_memory_.begin();
		it != barcode_memory_.end(); ++it)
	{
		if(ros::Time::now() > it->second)
		{
			NODELET_DEBUG_STREAM("Cleaned " << it->first << " from memory");
			barcode_memory_.erase(it);
		}
	}
  }

}
PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);
