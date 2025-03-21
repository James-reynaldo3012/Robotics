// Generated by gencpp from file rbo_create/ResetOdomResponse.msg
// DO NOT EDIT!


#ifndef RBO_CREATE_MESSAGE_RESETODOMRESPONSE_H
#define RBO_CREATE_MESSAGE_RESETODOMRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rbo_create
{
template <class ContainerAllocator>
struct ResetOdomResponse_
{
  typedef ResetOdomResponse_<ContainerAllocator> Type;

  ResetOdomResponse_()
    : success(false)  {
    }
  ResetOdomResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::rbo_create::ResetOdomResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbo_create::ResetOdomResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ResetOdomResponse_

typedef ::rbo_create::ResetOdomResponse_<std::allocator<void> > ResetOdomResponse;

typedef boost::shared_ptr< ::rbo_create::ResetOdomResponse > ResetOdomResponsePtr;
typedef boost::shared_ptr< ::rbo_create::ResetOdomResponse const> ResetOdomResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbo_create::ResetOdomResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rbo_create::ResetOdomResponse_<ContainerAllocator1> & lhs, const ::rbo_create::ResetOdomResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rbo_create::ResetOdomResponse_<ContainerAllocator1> & lhs, const ::rbo_create::ResetOdomResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rbo_create

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbo_create::ResetOdomResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbo_create::ResetOdomResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbo_create::ResetOdomResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::rbo_create::ResetOdomResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbo_create/ResetOdomResponse";
  }

  static const char* value(const ::rbo_create::ResetOdomResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::rbo_create::ResetOdomResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ResetOdomResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbo_create::ResetOdomResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rbo_create::ResetOdomResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RBO_CREATE_MESSAGE_RESETODOMRESPONSE_H
