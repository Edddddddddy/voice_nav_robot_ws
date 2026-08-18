// Copyright 2026 Edddddddddy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "one_shot_recognizer_proxy.hpp"

#include <utility>

namespace voice_nav_audio
{

OneShotRecognizerProxy::OneShotRecognizerProxy(OneShotRecognizerFactory & factory) noexcept
: factory_(factory)
{
}

OneShotRecognizerProxy::~OneShotRecognizerProxy()
{
  retire_child();
}

bool OneShotRecognizerProxy::start_turn() noexcept
{
  retire_child();
  try {
    child_ = factory_.create_armed();
  } catch (...) {
    child_.reset();
  }
  return child_ != nullptr;
}

void OneShotRecognizerProxy::shutdown() noexcept
{
  retire_child();
}

void OneShotRecognizerProxy::finish_input() noexcept
{
  if (child_ != nullptr) {
    child_->finish_input();
  }
}

void OneShotRecognizerProxy::process_frame(
  const CleanedAudioFrame & frame, SpeechEventSink & sink) noexcept
{
  if (child_ != nullptr) {
    child_->process_frame(frame, sink);
  }
}

void OneShotRecognizerProxy::on_turn_scope_opened(const TurnScopeIdentity & scope) noexcept
{
  if (child_ != nullptr) {
    child_->on_turn_scope_opened(scope);
  }
}

void OneShotRecognizerProxy::on_turn_scope_retired(const TurnScopeIdentity & scope) noexcept
{
  if (child_ != nullptr) {
    child_->on_turn_scope_retired(scope);
  }
}

void OneShotRecognizerProxy::retire_child() noexcept
{
  if (child_ == nullptr) {
    return;
  }
  child_->shutdown();
  child_.reset();
}

}  // namespace voice_nav_audio
