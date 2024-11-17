// Generated by gencpp from file lab_config/GetRangeResponse.msg
// DO NOT EDIT!


#ifndef LAB_CONFIG_MESSAGE_GETRANGERESPONSE_H
#define LAB_CONFIG_MESSAGE_GETRANGERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lab_config
{
template <class ContainerAllocator>
struct GetRangeResponse_
{
  typedef GetRangeResponse_<ContainerAllocator> Type;

  GetRangeResponse_()
    : success(false)
    , min()
    , max()  {
    }
  GetRangeResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , min(_alloc)
    , max(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::vector<int16_t, typename ContainerAllocator::template rebind<int16_t>::other >  _min_type;
  _min_type min;

   typedef std::vector<int16_t, typename ContainerAllocator::template rebind<int16_t>::other >  _max_type;
  _max_type max;





  typedef boost::shared_ptr< ::lab_config::GetRangeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lab_config::GetRangeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetRangeResponse_

typedef ::lab_config::GetRangeResponse_<std::allocator<void> > GetRangeResponse;

typedef boost::shared_ptr< ::lab_config::GetRangeResponse > GetRangeResponsePtr;
typedef boost::shared_ptr< ::lab_config::GetRangeResponse const> GetRangeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lab_config::GetRangeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lab_config::GetRangeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lab_config::GetRangeResponse_<ContainerAllocator1> & lhs, const ::lab_config::GetRangeResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.min == rhs.min &&
    lhs.max == rhs.max;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lab_config::GetRangeResponse_<ContainerAllocator1> & lhs, const ::lab_config::GetRangeResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lab_config

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::lab_config::GetRangeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lab_config::GetRangeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lab_config::GetRangeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lab_config::GetRangeResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lab_config::GetRangeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lab_config::GetRangeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lab_config::GetRangeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d599c1159c0ce143f7e9828a688b9c8f";
  }

  static const char* value(const ::lab_config::GetRangeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd599c1159c0ce143ULL;
  static const uint64_t static_value2 = 0xf7e9828a688b9c8fULL;
};

template<class ContainerAllocator>
struct DataType< ::lab_config::GetRangeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lab_config/GetRangeResponse";
  }

  static const char* value(const ::lab_config::GetRangeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lab_config::GetRangeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"int16[] min\n"
"int16[] max\n"
;
  }

  static const char* value(const ::lab_config::GetRangeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lab_config::GetRangeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.min);
      stream.next(m.max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetRangeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lab_config::GetRangeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lab_config::GetRangeResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "min[]" << std::endl;
    for (size_t i = 0; i < v.min.size(); ++i)
    {
      s << indent << "  min[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.min[i]);
    }
    s << indent << "max[]" << std::endl;
    for (size_t i = 0; i < v.max.size(); ++i)
    {
      s << indent << "  max[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.max[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LAB_CONFIG_MESSAGE_GETRANGERESPONSE_H