/*
 * rib_converter_ndarray.h
 *
 *  Created on: Feb 23, 2018
 *      Author: cpbove
 */

#ifndef __RIBConverterNdArray_H
#define __RIBConverterNdArray_H


#include "rib_converter.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtlndarray.h"

// OpenIGTLink message files
#include "igtlStringMessage.h"


class RIBConverterNDArray : public RIBConverter<ros_igtl_bridge::igtlndarray>
{

public:
	RIBConverterNDArray();
	RIBConverterNDArray(ros::NodeHandle *nh);
	RIBConverterNDArray(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);

  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "POINT"; }

public:
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlndarray::ConstPtr & msg);
};


#endif /* __RIBConverterNdArray_H */
