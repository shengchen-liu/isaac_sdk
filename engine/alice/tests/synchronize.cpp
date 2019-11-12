/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/alice.hpp"
#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/tests/messages.hpp"
#include "engine/core/logger.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace alice {

// Publishes two streams with identical data and acqtime but with a time delay
class MyPublisher : public Codelet {
 public:
  void start() override {
    tickPeriodically();
  }
  void tick() override {
    Foo foo{42 + (int)getTickCount(), 3.1415};
    tx_foo1().publish(foo);
    Sleep(SecondsToNano(0.04));
    tx_foo2().publish(foo);
  }

  ISAAC_RAW_TX(Foo, foo1)
  ISAAC_RAW_TX(Foo, foo2)
};

// Receives two synchronized message streams and checks that synchronization works
class MySubscriber : public Codelet {
 public:
  void start() override {
    tickOnMessage(rx_foo1());
    tickOnMessage(rx_foo2());
    synchronize(rx_foo1(), rx_foo2());
  }
  void tick() override {
    ASSERT_TRUE(rx_foo1().available());
    ASSERT_TRUE(rx_foo2().available());
    const Foo& foo1 = rx_foo1().get();
    const Foo& foo2 = rx_foo2().get();
    // EXPECT_NEAR(foo1.n, 42 + getTickCount(), 1);
    EXPECT_EQ(foo2.n, foo1.n);
    // Sleep(SecondsToNano(0.07));
  }

  ISAAC_RAW_RX(Foo, foo1)
  ISAAC_RAW_RX(Foo, foo2)
};

// Receives multiple streams and checks that a codelet can tick on a message while synchronizing
// on some other channels.
class MySubscriber2 : public Codelet {
 public:
  void start() override {
    synchronize(rx_foo1(), rx_foo2());
    tickOnMessage(rx_foo3());
    count_ = 0;
  }
  void tick() override {
    ASSERT_TRUE(rx_foo1().available() == rx_foo2().available());
    ASSERT_TRUE(rx_foo3().available());
    count_ ++;
  }
  void stop() override {
    EXPECT_NEAR(count_, get_expected_foo3_count(), get_count_tolerance());
  }
  ISAAC_PARAM(int, expected_foo3_count)
  ISAAC_PARAM(int, count_tolerance)
  ISAAC_RAW_RX(Foo, foo1)
  ISAAC_RAW_RX(Foo, foo2)
  ISAAC_RAW_RX(Foo, foo3)

 private:
  int count_;
};

TEST(MessagePassing, Test1) {
  Application app;
  // create publisher and subscriber nodes
  Node* pub_node = app.createMessageNode("pub");
  auto* pub = pub_node->addComponent<MyPublisher>();
  pub->set_tick_period("50ms");
  Node* sub_node = app.createMessageNode("sub");
  auto* sub = sub_node->addComponent<MySubscriber>();
  Connect(pub->tx_foo1(), sub->rx_foo1());
  Connect(pub->tx_foo2(), sub->rx_foo2());
  // run for a while
  app.startWaitStop(1.00);
}

TEST(MessagePassing, Test2) {
  Application app;
  // create publisher and subscriber nodes
  Node* pub_node = app.createMessageNode("pub");
  auto* pub = pub_node->addComponent<MyPublisher>();
  pub->set_tick_period("50ms");
  Node* sub_node = app.createMessageNode("sub");
  auto* sub = sub_node->addComponent<MySubscriber2>();
  sub->set_expected_foo3_count(20);
  sub->set_count_tolerance(2);
  Connect(pub->tx_foo1(), sub->rx_foo3());
  // run for a while
  app.startWaitStop(1.00);
}

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::MyPublisher);
ISAAC_ALICE_REGISTER_CODELET(isaac::alice::MySubscriber);
ISAAC_ALICE_REGISTER_CODELET(isaac::alice::MySubscriber2);
