// generated from rosidl_typesupport_connext_cpp/resource/idl__dds_connext__type_support.cpp.em
// with input from ssafy_msgs:msg\TurtlebotStatus.idl
// generated code does not contain a copyright notice

#include <limits>
#include <stdexcept>

#include "ssafy_msgs/msg/turtlebot_status__rosidl_typesupport_connext_cpp.hpp"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_connext_cpp/identifier.hpp"
#include "rosidl_typesupport_connext_cpp/message_type_support.h"
#include "rosidl_typesupport_connext_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_connext_cpp/wstring_conversion.hpp"

// forward declaration of message dependencies and their conversion functions
namespace geometry_msgs
{
namespace msg
{
namespace dds_
{
class Twist_;
}  // namespace dds_

namespace typesupport_connext_cpp
{

bool convert_ros_message_to_dds(
  const geometry_msgs::msg::Twist &,
  geometry_msgs::msg::dds_::Twist_ &);
bool convert_dds_message_to_ros(
  const geometry_msgs::msg::dds_::Twist_ &,
  geometry_msgs::msg::Twist &);
}  // namespace typesupport_connext_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace ssafy_msgs
{

namespace msg
{

namespace typesupport_connext_cpp
{


DDS_TypeCode *
get_type_code__TurtlebotStatus()
{
  return ssafy_msgs::msg::dds_::TurtlebotStatus_TypeSupport::get_typecode();
}

bool
convert_ros_message_to_dds(
  const ssafy_msgs::msg::TurtlebotStatus & ros_message,
  ssafy_msgs::msg::dds_::TurtlebotStatus_ & dds_message)
{
  // member.name twist
  if (
    !geometry_msgs::msg::typesupport_connext_cpp::convert_ros_message_to_dds(
      ros_message.twist,
      dds_message.twist_))
  {
    return false;
  }

  // member.name power_supply_status
  dds_message.power_supply_status_ =
    ros_message.power_supply_status;

  // member.name battery_percentage
  dds_message.battery_percentage_ =
    ros_message.battery_percentage;

  // member.name can_use_hand
  dds_message.can_use_hand_ =
    ros_message.can_use_hand;

  // member.name can_put
  dds_message.can_put_ =
    ros_message.can_put;

  // member.name can_lift
  dds_message.can_lift_ =
    ros_message.can_lift;

  return true;
}

bool
convert_dds_message_to_ros(
  const ssafy_msgs::msg::dds_::TurtlebotStatus_ & dds_message,
  ssafy_msgs::msg::TurtlebotStatus & ros_message)
{
  // member.name twist
  if (
    !geometry_msgs::msg::typesupport_connext_cpp::convert_dds_message_to_ros(
      dds_message.twist_,
      ros_message.twist))
  {
    return false;
  }

  // member.name power_supply_status
  ros_message.power_supply_status =
    dds_message.power_supply_status_;

  // member.name battery_percentage
  ros_message.battery_percentage =
    dds_message.battery_percentage_;

  // member.name can_use_hand
  ros_message.can_use_hand =
    dds_message.can_use_hand_ == DDS_BOOLEAN_TRUE;

  // member.name can_put
  ros_message.can_put =
    dds_message.can_put_ == DDS_BOOLEAN_TRUE;

  // member.name can_lift
  ros_message.can_lift =
    dds_message.can_lift_ == DDS_BOOLEAN_TRUE;

  return true;
}

bool
to_cdr_stream__TurtlebotStatus(
  const void * untyped_ros_message,
  rcutils_uint8_array_t * cdr_stream)
{
  if (!cdr_stream) {
    return false;
  }
  if (!untyped_ros_message) {
    return false;
  }

  // cast the untyped to the known ros message
  const ssafy_msgs::msg::TurtlebotStatus & ros_message =
    *(const ssafy_msgs::msg::TurtlebotStatus *)untyped_ros_message;

  // create a respective connext dds type
  ssafy_msgs::msg::dds_::TurtlebotStatus_ * dds_message = ssafy_msgs::msg::dds_::TurtlebotStatus_TypeSupport::create_data();
  if (!dds_message) {
    return false;
  }

  // convert ros to dds
  if (!convert_ros_message_to_dds(ros_message, *dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (ssafy_msgs::msg::dds_::TurtlebotStatus_Plugin_serialize_to_cdr_buffer(
      NULL,
      &expected_length,
      dds_message) != RTI_TRUE)
  {
    fprintf(stderr, "failed to call ssafy_msgs::msg::dds_::TurtlebotStatus_Plugin_serialize_to_cdr_buffer()\n");
    return false;
  }
  cdr_stream->buffer_length = expected_length;
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (cdr_stream->buffer_capacity < cdr_stream->buffer_length) {
    cdr_stream->allocator.deallocate(cdr_stream->buffer, cdr_stream->allocator.state);
    cdr_stream->buffer = static_cast<uint8_t *>(cdr_stream->allocator.allocate(cdr_stream->buffer_length, cdr_stream->allocator.state));
  }
  // call the function again and fill the buffer this time
  unsigned int buffer_length_uint = static_cast<unsigned int>(cdr_stream->buffer_length);
  if (ssafy_msgs::msg::dds_::TurtlebotStatus_Plugin_serialize_to_cdr_buffer(
      reinterpret_cast<char *>(cdr_stream->buffer),
      &buffer_length_uint,
      dds_message) != RTI_TRUE)
  {
    return false;
  }
  if (ssafy_msgs::msg::dds_::TurtlebotStatus_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return true;
}

bool
to_message__TurtlebotStatus(
  const rcutils_uint8_array_t * cdr_stream,
  void * untyped_ros_message)
{
  if (!cdr_stream) {
    return false;
  }
  if (!cdr_stream->buffer) {
    fprintf(stderr, "cdr stream doesn't contain data\n");
  }
  if (!untyped_ros_message) {
    return false;
  }

  ssafy_msgs::msg::dds_::TurtlebotStatus_ * dds_message =
    ssafy_msgs::msg::dds_::TurtlebotStatus_TypeSupport::create_data();
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (ssafy_msgs::msg::dds_::TurtlebotStatus_Plugin_deserialize_from_cdr_buffer(
      dds_message,
      reinterpret_cast<char *>(cdr_stream->buffer),
      static_cast<unsigned int>(cdr_stream->buffer_length)) != RTI_TRUE)
  {
    fprintf(stderr, "deserialize from cdr buffer failed\n");
    return false;
  }

  ssafy_msgs::msg::TurtlebotStatus & ros_message =
    *(ssafy_msgs::msg::TurtlebotStatus *)untyped_ros_message;
  bool success = convert_dds_message_to_ros(*dds_message, ros_message);
  if (ssafy_msgs::msg::dds_::TurtlebotStatus_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

static message_type_support_callbacks_t _TurtlebotStatus__callbacks = {
  "ssafy_msgs::msg",
  "TurtlebotStatus",
  &get_type_code__TurtlebotStatus,
  nullptr,
  nullptr,
  &to_cdr_stream__TurtlebotStatus,
  &to_message__TurtlebotStatus
};

static rosidl_message_type_support_t _TurtlebotStatus__handle = {
  rosidl_typesupport_connext_cpp::typesupport_identifier,
  &_TurtlebotStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_connext_cpp

}  // namespace msg

}  // namespace ssafy_msgs


namespace rosidl_typesupport_connext_cpp
{

template<>
ROSIDL_TYPESUPPORT_CONNEXT_CPP_EXPORT_ssafy_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<ssafy_msgs::msg::TurtlebotStatus>()
{
  return &ssafy_msgs::msg::typesupport_connext_cpp::_TurtlebotStatus__handle;
}

}  // namespace rosidl_typesupport_connext_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  ssafy_msgs, msg,
  TurtlebotStatus)()
{
  return &ssafy_msgs::msg::typesupport_connext_cpp::_TurtlebotStatus__handle;
}

#ifdef __cplusplus
}
#endif
