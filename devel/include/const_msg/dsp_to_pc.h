// Generated by gencpp from file const_msg/dsp_to_pc.msg
// DO NOT EDIT!


#ifndef CONST_MSG_MESSAGE_DSP_TO_PC_H
#define CONST_MSG_MESSAGE_DSP_TO_PC_H


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
struct dsp_to_pc_
{
  typedef dsp_to_pc_<ContainerAllocator> Type;

  dsp_to_pc_()
    : Header()
    , XDist(0)
    , YDist(0)
    , fAngle(0.0)
    , Vx(0)
    , Vy(0)
    , dw(0)
    , RecData_State(0)  {
    }
  dsp_to_pc_(const ContainerAllocator& _alloc)
    : Header(_alloc)
    , XDist(0)
    , YDist(0)
    , fAngle(0.0)
    , Vx(0)
    , Vy(0)
    , dw(0)
    , RecData_State(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _Header_type;
  _Header_type Header;

   typedef int32_t _XDist_type;
  _XDist_type XDist;

   typedef int32_t _YDist_type;
  _YDist_type YDist;

   typedef float _fAngle_type;
  _fAngle_type fAngle;

   typedef int16_t _Vx_type;
  _Vx_type Vx;

   typedef int16_t _Vy_type;
  _Vy_type Vy;

   typedef int16_t _dw_type;
  _dw_type dw;

   typedef uint8_t _RecData_State_type;
  _RecData_State_type RecData_State;





  typedef boost::shared_ptr< ::const_msg::dsp_to_pc_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::const_msg::dsp_to_pc_<ContainerAllocator> const> ConstPtr;

}; // struct dsp_to_pc_

typedef ::const_msg::dsp_to_pc_<std::allocator<void> > dsp_to_pc;

typedef boost::shared_ptr< ::const_msg::dsp_to_pc > dsp_to_pcPtr;
typedef boost::shared_ptr< ::const_msg::dsp_to_pc const> dsp_to_pcConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::const_msg::dsp_to_pc_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::const_msg::dsp_to_pc_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::const_msg::dsp_to_pc_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::const_msg::dsp_to_pc_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::const_msg::dsp_to_pc_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::const_msg::dsp_to_pc_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::const_msg::dsp_to_pc_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::const_msg::dsp_to_pc_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::const_msg::dsp_to_pc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d5bfc4d9d173fac3e9f429b7263af1d7";
  }

  static const char* value(const ::const_msg::dsp_to_pc_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd5bfc4d9d173fac3ULL;
  static const uint64_t static_value2 = 0xe9f429b7263af1d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::const_msg::dsp_to_pc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "const_msg/dsp_to_pc";
  }

  static const char* value(const ::const_msg::dsp_to_pc_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::const_msg::dsp_to_pc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header Header\n\
int32 XDist\n\
int32 YDist\n\
float32 fAngle\n\
int16 Vx\n\
int16 Vy\n\
int16 dw\n\
uint8 RecData_State\n\
 \n\
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

  static const char* value(const ::const_msg::dsp_to_pc_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::const_msg::dsp_to_pc_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Header);
      stream.next(m.XDist);
      stream.next(m.YDist);
      stream.next(m.fAngle);
      stream.next(m.Vx);
      stream.next(m.Vy);
      stream.next(m.dw);
      stream.next(m.RecData_State);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct dsp_to_pc_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::const_msg::dsp_to_pc_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::const_msg::dsp_to_pc_<ContainerAllocator>& v)
  {
    s << indent << "Header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.Header);
    s << indent << "XDist: ";
    Printer<int32_t>::stream(s, indent + "  ", v.XDist);
    s << indent << "YDist: ";
    Printer<int32_t>::stream(s, indent + "  ", v.YDist);
    s << indent << "fAngle: ";
    Printer<float>::stream(s, indent + "  ", v.fAngle);
    s << indent << "Vx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.Vx);
    s << indent << "Vy: ";
    Printer<int16_t>::stream(s, indent + "  ", v.Vy);
    s << indent << "dw: ";
    Printer<int16_t>::stream(s, indent + "  ", v.dw);
    s << indent << "RecData_State: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.RecData_State);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONST_MSG_MESSAGE_DSP_TO_PC_H
