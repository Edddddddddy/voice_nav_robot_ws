#include <cstdint>
#include <initializer_list>
#include <string>
#include <type_traits>

#include <gtest/gtest.h>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp>

#include <voice_nav_interfaces/action/speak.hpp>
#include <voice_nav_interfaces/msg/voice_turn.hpp>

namespace
{

using VoiceTurn = voice_nav_interfaces::msg::VoiceTurn;
using Speak = voice_nav_interfaces::action::Speak;
using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64;
using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8;

static_assert(std::is_same_v<std::remove_cv_t<decltype(VoiceTurn::COMMAND)>, uint8_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(VoiceTurn::STOP)>, uint8_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(Speak::Goal::NORMAL)>, uint8_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(Speak::Goal::URGENT)>, uint8_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(Speak::Result::COMPLETED)>, uint16_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(Speak::Result::CANCELED)>, uint16_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(Speak::Result::BARGED_IN)>, uint16_t>);
static_assert(std::is_same_v<std::remove_cv_t<decltype(Speak::Result::FAILED)>, uint16_t>);
static_assert(VoiceTurn::COMMAND == 1);
static_assert(VoiceTurn::STOP == 2);
static_assert(Speak::Goal::NORMAL == 1);
static_assert(Speak::Goal::URGENT == 2);
static_assert(Speak::Result::COMPLETED == 0);
static_assert(Speak::Result::CANCELED == 1);
static_assert(Speak::Result::BARGED_IN == 2);
static_assert(Speak::Result::FAILED == 10);

template<typename Message>
const MessageMembers & message_members()
{
  const auto * handle =
    rosidl_typesupport_introspection_cpp::get_message_type_support_handle<Message>();
  return *static_cast<const MessageMembers *>(handle->data);
}

struct FieldExpectation
{
  const char * name;
  uint8_t type;
  size_t string_upper_bound;
  const char * nested_namespace;
  const char * nested_name;
};

void expect_fields(
  const MessageMembers & members,
  std::initializer_list<FieldExpectation> expected)
{
  ASSERT_EQ(members.member_count_, expected.size());
  size_t index = 0;
  for (const auto & expectation : expected) {
    const auto & member = members.members_[index++];
    EXPECT_STREQ(member.name_, expectation.name);
    EXPECT_EQ(member.type_id_, expectation.type);
    EXPECT_EQ(member.string_upper_bound_, expectation.string_upper_bound);
    EXPECT_FALSE(member.is_array_);

    if (expectation.nested_namespace != nullptr) {
      ASSERT_NE(member.members_, nullptr);
      const auto * nested_members =
        static_cast<const MessageMembers *>(member.members_->data);
      EXPECT_STREQ(nested_members->message_namespace_, expectation.nested_namespace);
      EXPECT_STREQ(nested_members->message_name_, expectation.nested_name);
    }
  }
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
  const auto & members = message_members<VoiceTurn>();
  expect_fields(members, {
      {"voice_instance_id", ROS_TYPE_STRING, 36U, nullptr, nullptr},
      {"voice_seq", ROS_TYPE_UINT64, 0U, nullptr, nullptr},
      {"session_id", ROS_TYPE_STRING, 36U, nullptr, nullptr},
      {"turn_id", ROS_TYPE_STRING, 36U, nullptr, nullptr},
      {"kind", ROS_TYPE_UINT8, 0U, nullptr, nullptr},
      {"text", ROS_TYPE_STRING, 512U, nullptr, nullptr},
      {"confidence", ROS_TYPE_FLOAT, 0U, nullptr, nullptr},
      {"during_playback", ROS_TYPE_BOOLEAN, 0U, nullptr, nullptr},
  });
}

TEST(VoiceInterfaceCppContract, SpeakTypesAreConstructibleAndConstantsAreComplete)
{
  Speak::Goal goal;
  goal.source_instance_id = "voice-instance";
  goal.source_seq = 8;
  goal.session_id = "session";
  goal.turn_id = "turn";
  goal.priority = Speak::Goal::NORMAL;
  goal.text = "正在前往目标";
  goal.allow_barge_in = true;

  Speak::Result result;
  result.code = Speak::Result::COMPLETED;
  result.detail = "completed";

  Speak::Feedback feedback;
  feedback.played.sec = 3;
  feedback.played.nanosec = 500000000;

  EXPECT_EQ(Speak::Goal::NORMAL, 1);
  EXPECT_EQ(Speak::Goal::URGENT, 2);
  EXPECT_EQ(Speak::Result::COMPLETED, 0);
  EXPECT_EQ(Speak::Result::CANCELED, 1);
  EXPECT_EQ(Speak::Result::BARGED_IN, 2);
  EXPECT_EQ(Speak::Result::FAILED, 10);
  EXPECT_TRUE(goal.allow_barge_in);
  EXPECT_EQ(result.code, Speak::Result::COMPLETED);
  EXPECT_EQ(feedback.played.sec, 3);
}

TEST(VoiceInterfaceCppContract, SpeakGoalResultFeedbackFieldsAreOrderedAndBounded)
{
  expect_fields(message_members<Speak::Goal>(), {
      {"source_instance_id", ROS_TYPE_STRING, 36U, nullptr, nullptr},
      {"source_seq", ROS_TYPE_UINT64, 0U, nullptr, nullptr},
      {"session_id", ROS_TYPE_STRING, 36U, nullptr, nullptr},
      {"turn_id", ROS_TYPE_STRING, 36U, nullptr, nullptr},
      {"priority", ROS_TYPE_UINT8, 0U, nullptr, nullptr},
      {"text", ROS_TYPE_STRING, 512U, nullptr, nullptr},
      {"allow_barge_in", ROS_TYPE_BOOLEAN, 0U, nullptr, nullptr},
  });

  expect_fields(message_members<Speak::Result>(), {
      {"code", ROS_TYPE_UINT16, 0U, nullptr, nullptr},
      {"detail", ROS_TYPE_STRING, 160U, nullptr, nullptr},
  });

  expect_fields(message_members<Speak::Feedback>(), {
      {"played", ROS_TYPE_MESSAGE, 0U, "builtin_interfaces::msg", "Duration"},
  });
}

}  // namespace
