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
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"


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
        depth_img_Subscriber = nh.subscribe("camera/depth/image", 2, &QrDetectorNodelet::depthCb, this);
    }
}

void QrDetectorNodelet::disconnectCb()
{
    if (tagsPublisher.getNumSubscribers() == 0)
    {
        NODELET_INFO("Unsubscribing from image topic.");
        imgSubscriber.shutdown();
        depth_img_Subscriber.shutdown();
    }
}

void QrDetectorNodelet::imageCb(const sensor_msgs::ImageConstPtr &image)
{        
    cv_bridge::CvImageConstPtr cv_image;
	
	cv_image = cv_bridge::toCvShare(image, "mono8");

	zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data, cv_image->image.cols * cv_image->image.rows);
		
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
		
		std_msgs::String barcode_string;
	        barcode_string.data = barcode;

	        tagsPublisher.publish(barcode_string);
	    }
	zbar_image.set_data(NULL, 0);
  }
  
void QrDetectorNodelet::depthCb(const sensor_msgs::ImageConstPtr& image){
    cv_bridge::CvImagePtr cv_ptr;//line added for depth
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1); //line added for depth
   
    int front_depth = cv_ptr->image.at<short int>(cv::Point(240,320)); //line added for depth
	ROS_INFO_STREAM("Depth of front QR: " <<front_depth);
    
    

    //int back_depth = cv_ptr->image.at<short int>(cv::Point(240,320)); //line added for depth
    //ROS_INFO_STREAM("Depth of back QR: " <<back_depth);

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
