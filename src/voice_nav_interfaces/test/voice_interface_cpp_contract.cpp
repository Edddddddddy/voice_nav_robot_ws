#include <cstdint>
#include <string>
#include <type_traits>

#include <gtest/gtest.h>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>

#include <voice_nav_interfaces/msg/voice_turn.hpp>

namespace
{

using VoiceTurn = voice_nav_interfaces::msg::VoiceTurn;
using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8;

static_assert(std::is_same_v<std::remove_cv_t<decltype(VoiceTurn::COMMAND)>, uint8_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(VoiceTurn::STOP)>, uint8_t>);
static_assert(VoiceTurn::COMMAND == 1);
static_assert(VoiceTurn::STOP == 2);

const MessageMembers & message_members()
{
  const auto * handle =
    rosidl_typesupport_introspection_cpp::get_message_type_support_handle<VoiceTurn>();
  return *static_cast<const MessageMembers *>(handle->data);
}

TEST(VoiceInterfaceCppContract, VoiceTurnConstantsAndConstruction)
{
  VoiceTurn turn;
  turn.voice_instance_id = "voice-instance";
  turn.voice_seq = 7;
  turn.session_id = "session";
  turn.turn_id = "turn";
  turn.kind = VoiceTurn::COMMAND;
  turn.text = "向前走两米";
  turn.confidence = 0.95F;
  turn.during_playback = false;

  EXPECT_EQ(VoiceTurn::COMMAND, 1);
  EXPECT_EQ(VoiceTurn::STOP, 2);
  EXPECT_EQ(turn.kind, VoiceTurn::COMMAND);
  EXPECT_EQ(turn.text, "向前走两米");
}

TEST(VoiceInterfaceCppContract, VoiceTurnFieldsAreOrderedAndBounded)
{
  const auto & members = message_members();
  ASSERT_EQ(members.member_count_, 8U);

  const std::string names[] = {
    "voice_instance_id", "voice_seq", "session_id", "turn_id",
    "kind", "text", "confidence", "during_playback",
  };
  const uint8_t types[] = {
    ROS_TYPE_STRING, ROS_TYPE_UINT64, ROS_TYPE_STRING, ROS_TYPE_STRING,
    ROS_TYPE_UINT8, ROS_TYPE_STRING, ROS_TYPE_FLOAT, ROS_TYPE_BOOLEAN,
  };
  const size_t bounds[] = {36U, 0U, 36U, 36U, 0U, 512U, 0U, 0U};

  for (size_t index = 0; index < members.member_count_; ++index) {
    EXPECT_EQ(members.members_[index].name_, names[index]);
    EXPECT_EQ(members.members_[index].type_id_, types[index]);
    EXPECT_EQ(members.members_[index].string_upper_bound_, bounds[index]);
    EXPECT_FALSE(members.members_[index].is_array_);
  }
}

}  // namespace
