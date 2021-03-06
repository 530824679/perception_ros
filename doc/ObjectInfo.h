// Generated by gencpp from file perception_ros/ObjectInfo.msg
// DO NOT EDIT!


#ifndef PERCEPTION_ROS_MESSAGE_OBJECTINFO_H
#define PERCEPTION_ROS_MESSAGE_OBJECTINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace perception_ros
{
template <class ContainerAllocator>
struct ObjectInfo_
{
  typedef ObjectInfo_<ContainerAllocator> Type;

  ObjectInfo_()
    : id(0)
    , type(0)
    , yaw(0)
    , confidence(0)
    , height(0)
    , width(0)
    , length(0)
    , distance_xv(0)
    , distance_yv(0)
    , velocity_xv(0)
    , velocity_yv(0)
    , accelerate_xv(0)
    , accelerate_yv(0)
    , motion_state(0)  {
    }
  ObjectInfo_(const ContainerAllocator& _alloc)
    : id(0)
    , type(0)
    , yaw(0)
    , confidence(0)
    , height(0)
    , width(0)
    , length(0)
    , distance_xv(0)
    , distance_yv(0)
    , velocity_xv(0)
    , velocity_yv(0)
    , accelerate_xv(0)
    , accelerate_yv(0)
    , motion_state(0)  {
  (void)_alloc;
    }



   typedef uint16_t _id_type;
  _id_type id;

   typedef uint8_t _type_type;
  _type_type type;

   typedef uint8_t _yaw_type;
  _yaw_type yaw;

   typedef uint8_t _confidence_type;
  _confidence_type confidence;

   typedef uint16_t _height_type;
  _height_type height;

   typedef uint16_t _width_type;
  _width_type width;

   typedef uint16_t _length_type;
  _length_type length;

   typedef int16_t _distance_xv_type;
  _distance_xv_type distance_xv;

   typedef int16_t _distance_yv_type;
  _distance_yv_type distance_yv;

   typedef int16_t _velocity_xv_type;
  _velocity_xv_type velocity_xv;

   typedef int16_t _velocity_yv_type;
  _velocity_yv_type velocity_yv;

   typedef int16_t _accelerate_xv_type;
  _accelerate_xv_type accelerate_xv;

   typedef int16_t _accelerate_yv_type;
  _accelerate_yv_type accelerate_yv;

   typedef uint8_t _motion_state_type;
  _motion_state_type motion_state;





  typedef boost::shared_ptr< ::perception_ros::ObjectInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::perception_ros::ObjectInfo_<ContainerAllocator> const> ConstPtr;

}; // struct ObjectInfo_

typedef ::perception_ros::ObjectInfo_<std::allocator<void> > ObjectInfo;

typedef boost::shared_ptr< ::perception_ros::ObjectInfo > ObjectInfoPtr;
typedef boost::shared_ptr< ::perception_ros::ObjectInfo const> ObjectInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::perception_ros::ObjectInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::perception_ros::ObjectInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace perception_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'perception_ros': ['/HDD_Disk/catkin_ws/src/perception_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::perception_ros::ObjectInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::perception_ros::ObjectInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::perception_ros::ObjectInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::perception_ros::ObjectInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::perception_ros::ObjectInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::perception_ros::ObjectInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::perception_ros::ObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a84a83544761a25875cca8bb70b78846";
  }

  static const char* value(const ::perception_ros::ObjectInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa84a83544761a258ULL;
  static const uint64_t static_value2 = 0x75cca8bb70b78846ULL;
};

template<class ContainerAllocator>
struct DataType< ::perception_ros::ObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "perception_ros/ObjectInfo";
  }

  static const char* value(const ::perception_ros::ObjectInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::perception_ros::ObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16                       id                 # The No. of object\n\
uint8                        type 		        # The category of object\n\
# unknow     0\n\
# pedestrain 1\n\
# motor      2\n\
# car        3\n\
# truck      4\n\
\n\
uint8                        yaw                # The orientation angle of object\n\
uint8                        confidence         # The confidence of object\n\
uint16                       height             # The height of object\n\
uint16                       width              # The width of object\n\
uint16                       length             # The length of object\n\
int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate\n\
int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate\n\
int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate\n\
int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate\n\
int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate\n\
int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate\n\
uint8                        motion_state       # The motion status of object\n\
# unknow     0\n\
# moving     1\n\
# stationary 2\n\
\n\
\n\
\n\
";
  }

  static const char* value(const ::perception_ros::ObjectInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::perception_ros::ObjectInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.yaw);
      stream.next(m.confidence);
      stream.next(m.height);
      stream.next(m.width);
      stream.next(m.length);
      stream.next(m.distance_xv);
      stream.next(m.distance_yv);
      stream.next(m.velocity_xv);
      stream.next(m.velocity_yv);
      stream.next(m.accelerate_xv);
      stream.next(m.accelerate_yv);
      stream.next(m.motion_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObjectInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::perception_ros::ObjectInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::perception_ros::ObjectInfo_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "yaw: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.yaw);
    s << indent << "confidence: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.confidence);
    s << indent << "height: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.height);
    s << indent << "width: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.width);
    s << indent << "length: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.length);
    s << indent << "distance_xv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.distance_xv);
    s << indent << "distance_yv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.distance_yv);
    s << indent << "velocity_xv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.velocity_xv);
    s << indent << "velocity_yv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.velocity_yv);
    s << indent << "accelerate_xv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.accelerate_xv);
    s << indent << "accelerate_yv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.accelerate_yv);
    s << indent << "motion_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.motion_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PERCEPTION_ROS_MESSAGE_OBJECTINFO_H
