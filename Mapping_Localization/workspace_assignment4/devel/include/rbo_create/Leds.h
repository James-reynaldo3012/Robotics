// Generated by gencpp from file rbo_create/Leds.msg
// DO NOT EDIT!


#ifndef RBO_CREATE_MESSAGE_LEDS_H
#define RBO_CREATE_MESSAGE_LEDS_H

#include <ros/service_traits.h>


#include <rbo_create/LedsRequest.h>
#include <rbo_create/LedsResponse.h>


namespace rbo_create
{

struct Leds
{

typedef LedsRequest Request;
typedef LedsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Leds
} // namespace rbo_create


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rbo_create::Leds > {
  static const char* value()
  {
    return "926a8fe91d451f4d4bb099ec9e86ff14";
  }

  static const char* value(const ::rbo_create::Leds&) { return value(); }
};

template<>
struct DataType< ::rbo_create::Leds > {
  static const char* value()
  {
    return "rbo_create/Leds";
  }

  static const char* value(const ::rbo_create::Leds&) { return value(); }
};


// service_traits::MD5Sum< ::rbo_create::LedsRequest> should match
// service_traits::MD5Sum< ::rbo_create::Leds >
template<>
struct MD5Sum< ::rbo_create::LedsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rbo_create::Leds >::value();
  }
  static const char* value(const ::rbo_create::LedsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbo_create::LedsRequest> should match
// service_traits::DataType< ::rbo_create::Leds >
template<>
struct DataType< ::rbo_create::LedsRequest>
{
  static const char* value()
  {
    return DataType< ::rbo_create::Leds >::value();
  }
  static const char* value(const ::rbo_create::LedsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rbo_create::LedsResponse> should match
// service_traits::MD5Sum< ::rbo_create::Leds >
template<>
struct MD5Sum< ::rbo_create::LedsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rbo_create::Leds >::value();
  }
  static const char* value(const ::rbo_create::LedsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbo_create::LedsResponse> should match
// service_traits::DataType< ::rbo_create::Leds >
template<>
struct DataType< ::rbo_create::LedsResponse>
{
  static const char* value()
  {
    return DataType< ::rbo_create::Leds >::value();
  }
  static const char* value(const ::rbo_create::LedsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RBO_CREATE_MESSAGE_LEDS_H
