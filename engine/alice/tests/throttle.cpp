/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/alice.hpp"
#include "engine/alice/tests/foo_transmission.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace alice {

TEST(Alice, ThrottleBySignal) {
  Application app;
  // Create data publisher
  Node* data_node = app.createMessageNode("data");
  auto* data = data_node->addComponent<FooTransmitter>();
  data->set_tick_period("10ms");
  data->set_expected_tick_count(60);
  data->set_expected_tick_count_tolerance(6);
  // Create signal publisher
  Node* signal_node = app.createMessageNode("signal");
  auto* signal = signal_node->addComponent<FooTransmitter>();
  signal->set_tick_period("100ms");
  signal->set_expected_tick_count(6);
  signal->set_expected_tick_count_tolerance(1);
  // Create throttle
  Node* throttle_node = app.createMessageNode("throttle");
  auto* throttle = throttle_node->addComponent<Throttle>();
  throttle->set_data_channel("data");
  throttle->set_output_channel("output");
  throttle->set_minimum_interval(0.0);
  throttle->set_use_signal_channel(true);
  throttle->set_signal_channel("signal");
  throttle->set_acqtime_tolerance(10'000'000);
  throttle_node->getComponent<MessageLedger>()->set_history(20);
  // Create subscriber
  Node* sub_node = app.createMessageNode("sub");
  auto* sub = sub_node->addComponent<FooReceiver>();
  sub->set_expected_tick_count(6);
  sub->set_expected_tick_count_tolerance(1);
  sub->set_count_tolerance(1000);  // disable
  // Connections
  Connect(data->tx_foo(), throttle, "data");
  Connect(signal->tx_foo(), throttle, "signal");
  Connect(throttle, "output", sub->rx_foo());
  // run for a while
  app.startWaitStop(0.55);
}

TEST(Alice, ThrottleByTimer) {
  Application app;
  // Create data publisher
  Node* data_node = app.createMessageNode("data");
  auto* data = data_node->addComponent<FooTransmitter>();
  data->set_tick_period("10ms");
  data->set_expected_tick_count(60);
  data->set_expected_tick_count_tolerance(6);
  // Create throttle
  Node* throttle_node = app.createMessageNode("throttle");
  auto* throttle = throttle_node->addComponent<Throttle>();
  throttle->set_data_channel("data");
  throttle->set_output_channel("output");
  throttle->set_minimum_interval(0.1);
  throttle->set_use_signal_channel(false);
  // Create subscriber
  Node* sub_node = app.createMessageNode("sub");
  auto* sub = sub_node->addComponent<FooReceiver>();
  sub->set_expected_tick_count(6);
  sub->set_expected_tick_count_tolerance(1);
  sub->set_count_tolerance(1000);  // disable
  // Connections
  Connect(data->tx_foo(), throttle, "data");
  Connect(throttle, "output", sub->rx_foo());
  // run for a while
  app.startWaitStop(0.55);
}

}  // namespace alice
}  // namespace isaac
