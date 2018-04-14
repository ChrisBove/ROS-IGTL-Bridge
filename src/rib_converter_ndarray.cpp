/*
 * rib_converter_ndarray.cpp
 *
 *  Created on: Feb 23, 2018
 *      Author: cpbove
 */

#include "rib_converter_ndarray.h"
#include "rib_converter_manager.h"
#include "ros/ros.h"
#include "igtlNDArrayMessage.h"


RIBConverterNDArray::RIBConverterNDArray()
  : RIBConverter<ros_igtl_bridge::igtlndarray>()
{
}

RIBConverterNDArray::RIBConverterNDArray(ros::NodeHandle *nh)
  : RIBConverter<ros_igtl_bridge::igtlndarray>(nh)
{
}

RIBConverterNDArray::RIBConverterNDArray(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : RIBConverter<ros_igtl_bridge::igtlndarray>(topicPublish, topicSubscribe, nh)
{
}

int RIBConverterNDArray::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::NDArrayMessage::Pointer ndarrayMsg = igtl::NDArrayMessage::New();
  ndarrayMsg->SetMessageHeader(header);
  ndarrayMsg->AllocatePack();

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return 0;
    }

  socket->Receive(ndarrayMsg->GetPackBodyPointer(), ndarrayMsg->GetPackBodySize());
  int c = ndarrayMsg->Unpack(1);

  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0)
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Failed to unpack the message. Datatype: NDARRAY.");
    return 0;
    }

  igtl::ArrayBase::IndexType indexType = ndarrayMsg->GetArray()->GetSize();

  if (indexType.size() > 0) {
	  ros_igtl_bridge::igtlndarray msg;
	  msg.name = ndarrayMsg->GetDeviceName();
	  msg.scalar_type = ndarrayMsg->GetType();
	  msg.dim = ndarrayMsg->GetArray()->GetDimension();
	  msg.size = ndarrayMsg->GetArray()->GetSize();

	  igtl::ArrayBase* arrayBase = ndarrayMsg->GetArray();

	  // following https://github.com/openigtlink/OpenIGTLink/blob/master/Documents/Protocol/ndarray.md
	  switch(ndarrayMsg->GetType()) {
	  case igtl::NDArrayMessage::TYPE_INT8:
		  // TODO stuff the data into the message
		  break;
	  case igtl::NDArrayMessage::TYPE_UINT8:

		  break;
	  case igtl::NDArrayMessage::TYPE_INT16:

		  break;
	  case igtl::NDArrayMessage::TYPE_UINT16:

		  break;
	  case igtl::NDArrayMessage::TYPE_INT32:

		  break;
	  case igtl::NDArrayMessage::TYPE_UINT32:

		  break;
	  case igtl::NDArrayMessage::TYPE_FLOAT32:

		  break;
	  case igtl::NDArrayMessage::TYPE_FLOAT64:

		  break;
	  case igtl::NDArrayMessage::TYPE_COMPLEX:
		  ROS_ERROR("[ROS-IGTL-Bridge] Complex number handling not implemented.");
		  return 0;
		  break;
	  default:
		  ROS_ERROR("[ROS-IGTL-Bridge] Unrecognized scalar type of %u on NDARRAY.", ndarrayMsg->GetType());
		  return 0;
		  break;
	  }
    	this->publisher.publish(msg);
    }
  else
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Message NDARRAY has array with 0 elements.");
    return 0;
    }

  return 1;

}

void RIBConverterNDArray::onROSMessage(const ros_igtl_bridge::igtlndarray::ConstPtr & msg)
{
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return;
    }

  igtl::NDArrayMessage::Pointer ndArrayMsg = igtl::NDArrayMessage::New();
  ndArrayMsg->SetDeviceName(msg->name);

  igtl::ArrayBase* arrayBase;
  // TODO fill in the array based on type in msg

  switch(msg->scalar_type) {
  case igtl::NDArrayMessage::TYPE_INT8:
//	  igtl::Array<igtl_int8> array_int8;
//	  array_int8.SetSize()
//	  arrayBase->SetArray(&array_int8);
	  break;
  case igtl::NDArrayMessage::TYPE_UINT8:
//	  igtl::Array<igtl_uint8> array_uint8;
	  break;
  case igtl::NDArrayMessage::TYPE_INT16:

	  break;
  case igtl::NDArrayMessage::TYPE_UINT16:

	  break;
  case igtl::NDArrayMessage::TYPE_INT32:

	  break;
  case igtl::NDArrayMessage::TYPE_UINT32:

	  break;
  case igtl::NDArrayMessage::TYPE_FLOAT32:

	  break;
  case igtl::NDArrayMessage::TYPE_FLOAT64:

	  break;
  case igtl::NDArrayMessage::TYPE_COMPLEX:
	  ROS_ERROR("[ROS-IGTL-Bridge] Complex number handling not implemented.");
	  return;
	  break;
  default:
	  ROS_ERROR("[ROS-IGTL-Bridge] Unrecognized scalar type of %u on NDARRAY.", msg->scalar_type);
	  return;
	  break;
  }

  ndArrayMsg->SetArray(msg->scalar_type, arrayBase);
  ndArrayMsg->Pack();

  socket->Send(ndArrayMsg->GetPackPointer(), ndArrayMsg->GetPackSize());

}
