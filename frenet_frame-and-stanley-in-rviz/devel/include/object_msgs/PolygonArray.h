// Generated by gencpp from file object_msgs/PolygonArray.msg
// DO NOT EDIT!


#ifndef OBJECT_MSGS_MESSAGE_POLYGONARRAY_H
#define OBJECT_MSGS_MESSAGE_POLYGONARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <object_msgs/Polygon.h>

namespace object_msgs
{
template <class ContainerAllocator>
struct PolygonArray_
{
  typedef PolygonArray_<ContainerAllocator> Type;

  PolygonArray_()
    : polygon_list()  {
    }
  PolygonArray_(const ContainerAllocator& _alloc)
    : polygon_list(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::object_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::object_msgs::Polygon_<ContainerAllocator> >::other >  _polygon_list_type;
  _polygon_list_type polygon_list;





  typedef boost::shared_ptr< ::object_msgs::PolygonArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::object_msgs::PolygonArray_<ContainerAllocator> const> ConstPtr;

}; // struct PolygonArray_

typedef ::object_msgs::PolygonArray_<std::allocator<void> > PolygonArray;

typedef boost::shared_ptr< ::object_msgs::PolygonArray > PolygonArrayPtr;
typedef boost::shared_ptr< ::object_msgs::PolygonArray const> PolygonArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::object_msgs::PolygonArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::object_msgs::PolygonArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace object_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'object_msgs': ['/home/jdy/project_ws/src/custom_msgs/object_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::object_msgs::PolygonArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::object_msgs::PolygonArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::object_msgs::PolygonArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::object_msgs::PolygonArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::object_msgs::PolygonArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::object_msgs::PolygonArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::object_msgs::PolygonArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8cca23aedf7880d762e94679f98a6bf6";
  }

  static const char* value(const ::object_msgs::PolygonArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8cca23aedf7880d7ULL;
  static const uint64_t static_value2 = 0x62e94679f98a6bf6ULL;
};

template<class ContainerAllocator>
struct DataType< ::object_msgs::PolygonArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "object_msgs/PolygonArray";
  }

  static const char* value(const ::object_msgs::PolygonArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::object_msgs::PolygonArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "object_msgs/Polygon[] polygon_list\n\
================================================================================\n\
MSG: object_msgs/Polygon\n\
int32 n\n\
float32[] x\n\
float32[] y\n\
";
  }

  static const char* value(const ::object_msgs::PolygonArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::object_msgs::PolygonArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.polygon_list);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PolygonArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::object_msgs::PolygonArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::object_msgs::PolygonArray_<ContainerAllocator>& v)
  {
    s << indent << "polygon_list[]" << std::endl;
    for (size_t i = 0; i < v.polygon_list.size(); ++i)
    {
      s << indent << "  polygon_list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::object_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.polygon_list[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBJECT_MSGS_MESSAGE_POLYGONARRAY_H
