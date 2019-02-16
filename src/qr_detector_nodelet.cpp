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

//PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);

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
		//currently to see the data being published run rostopic echo /qr_codes
		//Future work: will need to publish this data to the ODrives node to control the start up spin
		std_msgs::String barcode_string;
		barcode_string.data = barcode;
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
