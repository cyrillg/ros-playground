// Generated by gencpp from file robot_control/WheelSpeeds.msg
// DO NOT EDIT!


#ifndef ROBOT_CONTROL_MESSAGE_WHEELSPEEDS_H
#define ROBOT_CONTROL_MESSAGE_WHEELSPEEDS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_control
{
template <class ContainerAllocator>
struct WheelSpeeds_
{
  typedef WheelSpeeds_<ContainerAllocator> Type;

  WheelSpeeds_()
    : left(0.0)
    , right(0.0)  {
    }
  WheelSpeeds_(const ContainerAllocator& _alloc)
    : left(0.0)
    , right(0.0)  {
  (void)_alloc;
    }



   typedef double _left_type;
  _left_type left;

   typedef double _right_type;
  _right_type right;




  typedef boost::shared_ptr< ::robot_control::WheelSpeeds_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_control::WheelSpeeds_<ContainerAllocator> const> ConstPtr;

}; // struct WheelSpeeds_

typedef ::robot_control::WheelSpeeds_<std::allocator<void> > WheelSpeeds;

typedef boost::shared_ptr< ::robot_control::WheelSpeeds > WheelSpeedsPtr;
typedef boost::shared_ptr< ::robot_control::WheelSpeeds const> WheelSpeedsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_control::WheelSpeeds_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_control::WheelSpeeds_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'robot_control': ['/home/cyrill/ros-playground/src/robot_control/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_control::WheelSpeeds_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_control::WheelSpeeds_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_control::WheelSpeeds_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_control::WheelSpeeds_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_control::WheelSpeeds_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_control::WheelSpeeds_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_control::WheelSpeeds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "50c2436c38cded221d061b57126c4e40";
  }

  static const char* value(const ::robot_control::WheelSpeeds_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x50c2436c38cded22ULL;
  static const uint64_t static_value2 = 0x1d061b57126c4e40ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_control::WheelSpeeds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_control/WheelSpeeds";
  }

  static const char* value(const ::robot_control::WheelSpeeds_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_control::WheelSpeeds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 left\n\
float64 right\n\
";
  }

  static const char* value(const ::robot_control::WheelSpeeds_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_control::WheelSpeeds_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left);
      stream.next(m.right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelSpeeds_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_control::WheelSpeeds_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_control::WheelSpeeds_<ContainerAllocator>& v)
  {
    s << indent << "left: ";
    Printer<double>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<double>::stream(s, indent + "  ", v.right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_CONTROL_MESSAGE_WHEELSPEEDS_H
