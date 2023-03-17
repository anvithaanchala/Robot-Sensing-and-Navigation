// Generated by gencpp from file imu_driver/convert_to_quaternion.msg
// DO NOT EDIT!


#ifndef IMU_DRIVER_MESSAGE_CONVERT_TO_QUATERNION_H
#define IMU_DRIVER_MESSAGE_CONVERT_TO_QUATERNION_H

#include <ros/service_traits.h>


#include <imu_driver/convert_to_quaternionRequest.h>
#include <imu_driver/convert_to_quaternionResponse.h>


namespace imu_driver
{

struct convert_to_quaternion
{

typedef convert_to_quaternionRequest Request;
typedef convert_to_quaternionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct convert_to_quaternion
} // namespace imu_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::imu_driver::convert_to_quaternion > {
  static const char* value()
  {
    return "b218cfe499191b24c074175e474522de";
  }

  static const char* value(const ::imu_driver::convert_to_quaternion&) { return value(); }
};

template<>
struct DataType< ::imu_driver::convert_to_quaternion > {
  static const char* value()
  {
    return "imu_driver/convert_to_quaternion";
  }

  static const char* value(const ::imu_driver::convert_to_quaternion&) { return value(); }
};


// service_traits::MD5Sum< ::imu_driver::convert_to_quaternionRequest> should match
// service_traits::MD5Sum< ::imu_driver::convert_to_quaternion >
template<>
struct MD5Sum< ::imu_driver::convert_to_quaternionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::imu_driver::convert_to_quaternion >::value();
  }
  static const char* value(const ::imu_driver::convert_to_quaternionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::imu_driver::convert_to_quaternionRequest> should match
// service_traits::DataType< ::imu_driver::convert_to_quaternion >
template<>
struct DataType< ::imu_driver::convert_to_quaternionRequest>
{
  static const char* value()
  {
    return DataType< ::imu_driver::convert_to_quaternion >::value();
  }
  static const char* value(const ::imu_driver::convert_to_quaternionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::imu_driver::convert_to_quaternionResponse> should match
// service_traits::MD5Sum< ::imu_driver::convert_to_quaternion >
template<>
struct MD5Sum< ::imu_driver::convert_to_quaternionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::imu_driver::convert_to_quaternion >::value();
  }
  static const char* value(const ::imu_driver::convert_to_quaternionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::imu_driver::convert_to_quaternionResponse> should match
// service_traits::DataType< ::imu_driver::convert_to_quaternion >
template<>
struct DataType< ::imu_driver::convert_to_quaternionResponse>
{
  static const char* value()
  {
    return DataType< ::imu_driver::convert_to_quaternion >::value();
  }
  static const char* value(const ::imu_driver::convert_to_quaternionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // IMU_DRIVER_MESSAGE_CONVERT_TO_QUATERNION_H
