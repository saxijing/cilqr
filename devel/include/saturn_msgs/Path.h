// Generated by gencpp from file saturn_msgs/Path.msg
// DO NOT EDIT!


#ifndef SATURN_MSGS_MESSAGE_PATH_H
#define SATURN_MSGS_MESSAGE_PATH_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <saturn_msgs/StateLite.h>

namespace saturn_msgs
{
template <class ContainerAllocator>
struct Path_
{
  typedef Path_<ContainerAllocator> Type;

  Path_()
    : header()
    , path()  {
    }
  Path_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , path(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::saturn_msgs::StateLite_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::saturn_msgs::StateLite_<ContainerAllocator> >> _path_type;
  _path_type path;





  typedef boost::shared_ptr< ::saturn_msgs::Path_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::saturn_msgs::Path_<ContainerAllocator> const> ConstPtr;

}; // struct Path_

typedef ::saturn_msgs::Path_<std::allocator<void> > Path;

typedef boost::shared_ptr< ::saturn_msgs::Path > PathPtr;
typedef boost::shared_ptr< ::saturn_msgs::Path const> PathConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::saturn_msgs::Path_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::saturn_msgs::Path_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::saturn_msgs::Path_<ContainerAllocator1> & lhs, const ::saturn_msgs::Path_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.path == rhs.path;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::saturn_msgs::Path_<ContainerAllocator1> & lhs, const ::saturn_msgs::Path_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace saturn_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::saturn_msgs::Path_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::saturn_msgs::Path_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::saturn_msgs::Path_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::saturn_msgs::Path_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::saturn_msgs::Path_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::saturn_msgs::Path_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::saturn_msgs::Path_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5329b3c8e4d0d10f7d64d970ba2d2048";
  }

  static const char* value(const ::saturn_msgs::Path_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5329b3c8e4d0d10fULL;
  static const uint64_t static_value2 = 0x7d64d970ba2d2048ULL;
};

template<class ContainerAllocator>
struct DataType< ::saturn_msgs::Path_<ContainerAllocator> >
{
  static const char* value()
  {
    return "saturn_msgs/Path";
  }

  static const char* value(const ::saturn_msgs::Path_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::saturn_msgs::Path_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"StateLite[] path\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: saturn_msgs/StateLite\n"
"std_msgs/Header header\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
"float64 v\n"
"float64 accel\n"
"float64 yawrate\n"
;
  }

  static const char* value(const ::saturn_msgs::Path_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::saturn_msgs::Path_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Path_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::saturn_msgs::Path_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::saturn_msgs::Path_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "path[]" << std::endl;
    for (size_t i = 0; i < v.path.size(); ++i)
    {
      s << indent << "  path[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::saturn_msgs::StateLite_<ContainerAllocator> >::stream(s, indent + "    ", v.path[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SATURN_MSGS_MESSAGE_PATH_H
