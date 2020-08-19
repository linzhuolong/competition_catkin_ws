// Generated by gencpp from file const_msg/object_param.msg
// DO NOT EDIT!


#ifndef CONST_MSG_MESSAGE_OBJECT_PARAM_H
#define CONST_MSG_MESSAGE_OBJECT_PARAM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace const_msg
{
template <class ContainerAllocator>
struct object_param_
{
  typedef object_param_<ContainerAllocator> Type;

  object_param_()
    : Header()
    , obj_angle(0.0)
    , obj_distance(0)  {
    }
  object_param_(const ContainerAllocator& _alloc)
    : Header(_alloc)
    , obj_angle(0.0)
    , obj_distance(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _Header_type;
  _Header_type Header;

   typedef double _obj_angle_type;
  _obj_angle_type obj_angle;

   typedef int64_t _obj_distance_type;
  _obj_distance_type obj_distance;





  typedef boost::shared_ptr< ::const_msg::object_param_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::const_msg::object_param_<ContainerAllocator> const> ConstPtr;

}; // struct object_param_

typedef ::const_msg::object_param_<std::allocator<void> > object_param;

typedef boost::shared_ptr< ::const_msg::object_param > object_paramPtr;
typedef boost::shared_ptr< ::const_msg::object_param const> object_paramConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::const_msg::object_param_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::const_msg::object_param_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace const_msg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'const_msg': ['/home/bearli/competition_catkin_ws/src/const_msg/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::const_msg::object_param_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::const_msg::object_param_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::const_msg::object_param_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::const_msg::object_param_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::const_msg::object_param_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::const_msg::object_param_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::const_msg::object_param_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6cb701956f8e3917f84ab2b8286c0409";
  }

  static const char* value(const ::const_msg::object_param_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6cb701956f8e3917ULL;
  static const uint64_t static_value2 = 0xf84ab2b8286c0409ULL;
};

template<class ContainerAllocator>
struct DataType< ::const_msg::object_param_<ContainerAllocator> >
{
  static const char* value()
  {
    return "const_msg/object_param";
  }

  static const char* value(const ::const_msg::object_param_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::const_msg::object_param_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header Header\n\
float64 obj_angle\n\
int64 obj_distance\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::const_msg::object_param_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::const_msg::object_param_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Header);
      stream.next(m.obj_angle);
      stream.next(m.obj_distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct object_param_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::const_msg::object_param_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::const_msg::object_param_<ContainerAllocator>& v)
  {
    s << indent << "Header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.Header);
    s << indent << "obj_angle: ";
    Printer<double>::stream(s, indent + "  ", v.obj_angle);
    s << indent << "obj_distance: ";
    Printer<int64_t>::stream(s, indent + "  ", v.obj_distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONST_MSG_MESSAGE_OBJECT_PARAM_H
