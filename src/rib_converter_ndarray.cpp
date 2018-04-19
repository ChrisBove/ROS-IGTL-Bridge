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
	  igtl::ArrayBase::IndexType index(ndarrayMsg->GetArray()->GetSize());

	  // get total index size of array
	  unsigned length = 0;
	  for (unsigned i = 0; i < arrayBase->GetSize().size(); i++) {
		  length += arrayBase->GetSize().at(i);
	  }

	  std::vector<std::int8_t> int8_vector;
	  std::vector<std::uint8_t> uint8_vector;
	  std::vector<std::int16_t> int16_vector;
	  std::vector<std::uint16_t> uint16_vector;
	  std::vector<std::int32_t> int32_vector;
	  std::vector<std::uint32_t> uint32_vector;
	  std::vector<float> float32_vector;
	  std::vector<double> float64_vector;

	  igtl_int8* array_ptr_int8;
	  igtl_uint8* array_ptr_uint8;
	  igtl_int16* array_ptr_int16;
	  igtl_uint16* array_ptr_uint16;
	  igtl_int32* array_ptr_int32;
	  igtl_uint32* array_ptr_uint32;
	  igtl_float32* array_ptr_float32;
	  igtl_float64* array_ptr_float64;

	  // following https://github.com/openigtlink/OpenIGTLink/blob/master/Documents/Protocol/ndarray.md
	  switch(ndarrayMsg->GetType()) {
	  case igtl::NDArrayMessage::TYPE_INT8:
		  array_ptr_int8 = (igtl_int8 *) arrayBase->GetRawArray();
		  int8_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  int8_vector.push_back(array_ptr_int8[i]);
		  }

		  msg.data_int8 = int8_vector;
		  break;
	  case igtl::NDArrayMessage::TYPE_UINT8:
		  array_ptr_uint8 = (igtl_uint8 *) arrayBase->GetRawArray();
		  uint8_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  uint8_vector.push_back(array_ptr_uint8[i]);
		  }

		  msg.data_uint8 = uint8_vector;
		  break;
	  case igtl::NDArrayMessage::TYPE_INT16:
		  array_ptr_int16 = (igtl_int16 *) arrayBase->GetRawArray();
		  int16_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  int16_vector.push_back(array_ptr_int16[i]);
		  }

		  msg.data_int16 = int16_vector;
		  break;
	  case igtl::NDArrayMessage::TYPE_UINT16:
		  array_ptr_uint16 = (igtl_uint16 *) arrayBase->GetRawArray();
		  uint16_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  uint16_vector.push_back(array_ptr_uint16[i]);
		  }

		  msg.data_uint16 = uint16_vector;
		  break;
	  case igtl::NDArrayMessage::TYPE_INT32:
		  array_ptr_int32 = (igtl_int32 *) arrayBase->GetRawArray();
		  int32_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  int32_vector.push_back(array_ptr_int32[i]);
		  }

		  msg.data_int32 = int32_vector;

		  break;
	  case igtl::NDArrayMessage::TYPE_UINT32:
		  array_ptr_uint32 = (igtl_uint32 *) arrayBase->GetRawArray();
		  uint32_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  uint32_vector.push_back(array_ptr_uint32[i]);
		  }

		  msg.data_uint32 = uint32_vector;

		  break;
	  case igtl::NDArrayMessage::TYPE_FLOAT32:
		  array_ptr_float32 = (igtl_float32 *) arrayBase->GetRawArray();
		  float32_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  float32_vector.push_back(array_ptr_float32[i]);
		  }

		  msg.data_float32 = float32_vector;
		  break;
	  case igtl::NDArrayMessage::TYPE_FLOAT64:
		  array_ptr_float64 = (igtl_float64 *) arrayBase->GetRawArray();
		  float64_vector.reserve(length);
		  for (int i = 0; i < length; i++) {
			  float64_vector.push_back(array_ptr_float64[i]);
		  }

		  msg.data_float64 = float64_vector;
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

  // get total index size of array
  unsigned length = 0;
  for (unsigned i = 0; i < msg->size.size(); i++) {
	  length += msg->size.at(i);
  }

  igtl::NDArrayMessage::Pointer ndArrayMsg = igtl::NDArrayMessage::New();
  ndArrayMsg->SetDeviceName(msg->name);

  igtl::ArrayBase* arrayBase;

  igtl::Array<igtl_int8> array_int8;
  igtl::Array<igtl_uint8> array_uint8;
  igtl::Array<igtl_int16> array_int16;
  igtl::Array<igtl_uint16> array_uint16;
  igtl::Array<igtl_int32> array_int32;
  igtl::Array<igtl_uint32> array_uint32;
  igtl::Array<igtl_float32> array_float32;
  igtl::Array<igtl_float64> array_float64;

  igtl_int8 c_array_int8[length];
  igtl_uint8 c_array_uint8[length];
  igtl_int16 c_array_int16[length];
  igtl_uint16 c_array_uint16[length];
  igtl_int32 c_array_int32[length];
  igtl_uint32 c_array_uint32[length];
  igtl_float32 c_array_float32[length];
  igtl_float64 c_array_float64[length];

  switch(msg->scalar_type) {
  case igtl::NDArrayMessage::TYPE_INT8:
	  array_int8.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_int8[i] = (igtl_int8) msg->data_int8.at(i);
	  }

	  array_int8.SetArray((void*) c_array_int8);
	  ndArrayMsg->SetArray(msg->scalar_type, &array_int8);
	  break;
  case igtl::NDArrayMessage::TYPE_UINT8:
	  array_uint8.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_uint8[i] = (igtl_uint8) msg->data_uint8.at(i);
	  }

	  array_uint8.SetArray((void*) c_array_uint8);

	  ndArrayMsg->SetArray(msg->scalar_type, &array_uint8);
	  break;
  case igtl::NDArrayMessage::TYPE_INT16:
	  array_int16.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_int16[i] = (igtl_int16) msg->data_int16.at(i);
	  }

	  array_int16.SetArray((void*) c_array_int16);
	  ndArrayMsg->SetArray(msg->scalar_type, &array_int16);
	  break;
  case igtl::NDArrayMessage::TYPE_UINT16:
	  array_uint16.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_uint16[i] = (igtl_uint16) msg->data_uint16.at(i);
	  }

	  array_uint16.SetArray((void*) c_array_uint16);
	  ndArrayMsg->SetArray(msg->scalar_type, &array_uint16);
	  break;
  case igtl::NDArrayMessage::TYPE_INT32:
	  array_int32.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_int32[i] = (igtl_int32) msg->data_int32.at(i);
	  }

	  array_int32.SetArray((void*) c_array_int32);
	  ndArrayMsg->SetArray(msg->scalar_type, &array_int32);
	  break;
  case igtl::NDArrayMessage::TYPE_UINT32:
	  array_uint32.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_uint32[i] = (igtl_uint32) msg->data_uint32.at(i);
	  }

	  array_uint32.SetArray((void*) c_array_uint32);
	  ndArrayMsg->SetArray(msg->scalar_type, &array_uint32);
	  break;
  case igtl::NDArrayMessage::TYPE_FLOAT32:
	  array_float32.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_float32[i] = (igtl_float32) msg->data_float32.at(i);
	  }

	  array_float32.SetArray((void*) c_array_float32);
	  ndArrayMsg->SetArray(msg->scalar_type, &array_float32);
	  break;
  case igtl::NDArrayMessage::TYPE_FLOAT64:
	  array_float64.SetSize(msg->size);

	  for (int i = 0; i < length; i++) {
		  c_array_float64[i] = (igtl_float64) msg->data_float64.at(i);
	  }

	  array_float64.SetArray((void*) c_array_float64);
	  ndArrayMsg->SetArray(msg->scalar_type, &array_float64);
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

  ndArrayMsg->Pack();

  socket->Send(ndArrayMsg->GetPackPointer(), ndArrayMsg->GetPackSize());

}
