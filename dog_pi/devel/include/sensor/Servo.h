// Generated by gencpp from file sensor/Servo.msg
// DO NOT EDIT!


#ifndef SENSOR_MESSAGE_SERVO_H
#define SENSOR_MESSAGE_SERVO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sensor
{
template <class ContainerAllocator>
struct Servo_
{
  typedef Servo_<ContainerAllocator> Type;

  Servo_()
    : id(0)
    , pulse(0)
    , time(0)  {
    }
  Servo_(const ContainerAllocator& _alloc)
    : id(0)
    , pulse(0)
    , time(0)  {
  (void)_alloc;
    }



   typedef int8_t _id_type;
  _id_type id;

   typedef int32_t _pulse_type;
  _pulse_type pulse;

   typedef int32_t _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::sensor::Servo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sensor::Servo_<ContainerAllocator> const> ConstPtr;

}; // struct Servo_

typedef ::sensor::Servo_<std::allocator<void> > Servo;

typedef boost::shared_ptr< ::sensor::Servo > ServoPtr;
typedef boost::shared_ptr< ::sensor::Servo const> ServoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensor::Servo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensor::Servo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sensor::Servo_<ContainerAllocator1> & lhs, const ::sensor::Servo_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.pulse == rhs.pulse &&
    lhs.time == rhs.time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sensor::Servo_<ContainerAllocator1> & lhs, const ::sensor::Servo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sensor

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::sensor::Servo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensor::Servo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor::Servo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensor::Servo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor::Servo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensor::Servo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensor::Servo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "27fba4c6dd04ddcfa19ba8394a1086e6";
  }

  static const char* value(const ::sensor::Servo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x27fba4c6dd04ddcfULL;
  static const uint64_t static_value2 = 0xa19ba8394a1086e6ULL;
};

template<class ContainerAllocator>
struct DataType< ::sensor::Servo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor/Servo";
  }

  static const char* value(const ::sensor::Servo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensor::Servo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 id\n"
"int32 pulse\n"
"int32 time\n"
;
  }

  static const char* value(const ::sensor::Servo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensor::Servo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.pulse);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Servo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensor::Servo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensor::Servo_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int8_t>::stream(s, indent + "  ", v.id);
    s << indent << "pulse: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pulse);
    s << indent << "time: ";
    Printer<int32_t>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSOR_MESSAGE_SERVO_H