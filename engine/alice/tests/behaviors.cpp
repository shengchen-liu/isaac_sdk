/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/alice.hpp"
#include "engine/alice/behaviors/ConstantBehavior.hpp"
#include "engine/alice/behaviors/MemorySelectorBehavior.hpp"
#include "engine/alice/behaviors/MemorySequenceBehavior.hpp"
#include "engine/alice/behaviors/NodeGroup.hpp"
#include "engine/alice/behaviors/ParallelBehavior.hpp"
#include "engine/alice/behaviors/SwitchBehavior.hpp"
#include "engine/alice/behaviors/TimerBehavior.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace alice {

namespace {

// Creates a node with a behavior of given type
template <typename Behavior>
Behavior* CreateBehaviorNode(Application& app, const std::string& name,
                             const std::vector<std::string>& children) {
  Node* node = app.createNode(name);
  auto* node_group = node->addComponent<behaviors::NodeGroup>();
  node_group->set_node_names(children);
  return node->addComponent<Behavior>();
}

// Creates a node with a behavior returning the given status
behaviors::ConstantBehavior* CreateConstantBehaviorNode(Application& app, const std::string& name,
                                                        Status status) {
  Node* node = app.createNode(name);
  node->disable_automatic_start = true;
  auto* behavior = node->addComponent<behaviors::ConstantBehavior>();
  behavior->set_status(status);
  return behavior;
}

// Creates a node with a timer behavior
behaviors::TimerBehavior* CreateTimerBehaviorNode(Application& app, const std::string& name,
                                                  double delay, Status status) {
  Node* node = app.createNode(name);
  node->disable_automatic_start = true;
  auto* behavior = node->addComponent<behaviors::TimerBehavior>();
  behavior->set_delay(delay);
  behavior->set_status(status);
  return behavior;
}

void CreateConstantBehaviorTest(Status status) {
  Application app;
  Node* node = app.createNode("behavior");
  auto* behavior = node->addComponent<behaviors::ConstantBehavior>();
  behavior->set_status(status);
  app.startWaitStop(0.10);
  EXPECT_EQ(behavior->getStatus(), status);
}

void CreateTimerBehaviorTest(double delay, Status status, double app_duration,
                             Status expected_status) {
  Application app;
  Node* node = app.createNode("behavior");
  auto* behavior = node->addComponent<behaviors::TimerBehavior>();
  behavior->set_delay(delay);
  behavior->set_status(status);
  app.startWaitStop(app_duration);
  EXPECT_EQ(behavior->getStatus(), expected_status);
}

template <typename Behavior>
void CreateBehaviorTest(const std::vector<Status>& child_stati, Status expected_status) {
  std::vector<std::string> names;
  for (size_t i = 0; i < child_stati.size(); i++) {
    names.push_back("child_" + std::to_string(i));
  }
  Application app;
  auto* behavior = CreateBehaviorNode<Behavior>(
      app, "parent", names);
  for (size_t i = 0; i < child_stati.size(); i++) {
    CreateConstantBehaviorNode(app, names[i], child_stati[i]);
  }
  app.startWaitStop(0.10);
  EXPECT_EQ(behavior->getStatus(), expected_status);
}

template <typename Behavior>
void CreateBehaviorTest(const std::vector<std::pair<double,Status>>& child_data,
                        Status expected_status, double duration) {
  std::vector<std::string> names;
  for (size_t i = 0; i < child_data.size(); i++) {
    names.push_back("child_" + std::to_string(i));
  }
  Application app;
  auto* behavior = CreateBehaviorNode<Behavior>(
      app, "parent", names);
  for (size_t i = 0; i < child_data.size(); i++) {
    CreateTimerBehaviorNode(app, names[i], child_data[i].first, child_data[i].second);
  }
  app.startWaitStop(duration);
  EXPECT_EQ(behavior->getStatus(), expected_status);
}

}  // namespace

TEST(Behaviors, ConstantBehavior) {
  CreateConstantBehaviorTest(Status::SUCCESS);
  CreateConstantBehaviorTest(Status::FAILURE);
  CreateConstantBehaviorTest(Status::RUNNING);
}

TEST(Behaviors, TimerBehavior) {
  CreateTimerBehaviorTest(0.05, Status::SUCCESS, 0.15, Status::SUCCESS);
  CreateTimerBehaviorTest(0.05, Status::FAILURE, 0.15, Status::FAILURE);
  CreateTimerBehaviorTest(10.0, Status::FAILURE, 0.15, Status::RUNNING);
}

TEST(Behaviors, MemorySelectorBehavior) {
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>({Status::SUCCESS}, Status::SUCCESS);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>({Status::FAILURE}, Status::FAILURE);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>({Status::RUNNING}, Status::RUNNING);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>(
      {Status::RUNNING, Status::SUCCESS},Status::RUNNING);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>(
      {Status::FAILURE, Status::SUCCESS}, Status::SUCCESS);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>(
      {Status::FAILURE, Status::FAILURE, Status::SUCCESS}, Status::SUCCESS);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>(
      {Status::SUCCESS, Status::SUCCESS, Status::FAILURE}, Status::SUCCESS);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>(
      {Status::SUCCESS, Status::SUCCESS, Status::SUCCESS}, Status::SUCCESS);
  CreateBehaviorTest<behaviors::MemorySelectorBehavior>(
      {Status::FAILURE, Status::FAILURE, Status::FAILURE}, Status::FAILURE);
}

TEST(Behaviors, MemorySequenceBehavior) {
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>({Status::SUCCESS}, Status::SUCCESS);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>({Status::FAILURE}, Status::FAILURE);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>({Status::RUNNING}, Status::RUNNING);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>(
      {Status::RUNNING, Status::SUCCESS},Status::RUNNING);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>(
      {Status::FAILURE, Status::SUCCESS}, Status::FAILURE);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>(
      {Status::FAILURE, Status::FAILURE, Status::SUCCESS}, Status::FAILURE);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>(
      {Status::SUCCESS, Status::FAILURE, Status::SUCCESS}, Status::FAILURE);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>(
      {Status::SUCCESS, Status::SUCCESS, Status::FAILURE}, Status::FAILURE);
  CreateBehaviorTest<behaviors::MemorySequenceBehavior>(
      {Status::SUCCESS, Status::SUCCESS, Status::SUCCESS}, Status::SUCCESS);
}

TEST(Behaviors, ParallelBehavior) {
  CreateBehaviorTest<behaviors::ParallelBehavior>(
      {{0.05, Status::SUCCESS}, {0.10, Status::SUCCESS}}, Status::SUCCESS, 0.15);
  CreateBehaviorTest<behaviors::ParallelBehavior>(
      {{0.05, Status::SUCCESS}, {0.10, Status::FAILURE}}, Status::FAILURE, 0.15);
  CreateBehaviorTest<behaviors::ParallelBehavior>(
      {{0.05, Status::SUCCESS}, {10.0, Status::FAILURE}}, Status::RUNNING, 0.10);
  CreateBehaviorTest<behaviors::ParallelBehavior>(
      {{0.05, Status::SUCCESS}, {0.03, Status::SUCCESS}, {0.05, Status::SUCCESS},
       {0.06, Status::SUCCESS}, {0.09, Status::SUCCESS}, {0.04, Status::SUCCESS}},
      Status::SUCCESS, 0.15);
}

TEST(Behaviors, SwitchBehavior) {
  Application app;
  auto* behavior = CreateBehaviorNode<behaviors::SwitchBehavior>(app, "parent",
      {"child_0", "child_1", "child_2"});
  CreateTimerBehaviorNode(app, "child_0", 0.10, Status::SUCCESS);
  CreateTimerBehaviorNode(app, "child_1", 0.20, Status::FAILURE);
  CreateTimerBehaviorNode(app, "child_2", 0.30, Status::SUCCESS);
  app.start();
  Sleep(0.20);
  behavior->set_desired_behavior("child_1");
  Sleep(0.10);
  behavior->set_desired_behavior("child_0");
  Sleep(0.20);
  app.stop();
  EXPECT_EQ(behavior->getStatus(), Status::RUNNING);
}

TEST(Behaviors, Nested) {
  Application app;

  auto* top = CreateBehaviorNode<behaviors::MemorySequenceBehavior>(app, "top",
      {"0", "3", "1", "4", "2"});
  EXPECT_EQ(top->getStatus(), Status::RUNNING);

  CreateBehaviorNode<behaviors::MemorySelectorBehavior>(app, "0", {"00", "01", "02"});
  CreateTimerBehaviorNode(app, "00", 0.07, Status::FAILURE);
  CreateTimerBehaviorNode(app, "01", 0.06, Status::SUCCESS);
  CreateTimerBehaviorNode(app, "02", 0.09, Status::FAILURE);

  CreateBehaviorNode<behaviors::MemorySelectorBehavior>(app, "3", {"30", "31", "32"});
  CreateTimerBehaviorNode(app, "30", 0.07, Status::SUCCESS);
  CreateTimerBehaviorNode(app, "31", 0.11, Status::SUCCESS);
  CreateTimerBehaviorNode(app, "32", 0.05, Status::SUCCESS);

  CreateBehaviorNode<behaviors::MemorySequenceBehavior>(app, "1", {"10", "11", "12"});
  CreateTimerBehaviorNode(app, "10", 0.04, Status::SUCCESS);
  CreateTimerBehaviorNode(app, "11", 0.05, Status::SUCCESS);
  CreateTimerBehaviorNode(app, "12", 0.06, Status::SUCCESS);

  CreateTimerBehaviorNode(app, "4", 0.06, Status::SUCCESS);

  CreateBehaviorNode<behaviors::MemorySelectorBehavior>(app, "2", {"20", "21", "22"});
  CreateTimerBehaviorNode(app, "20", 0.03, Status::FAILURE);
  CreateTimerBehaviorNode(app, "21", 0.08, Status::FAILURE);
  CreateTimerBehaviorNode(app, "22", 0.05, Status::SUCCESS);

  app.startWaitStop(1.50);
  EXPECT_EQ(top->getStatus(), Status::SUCCESS);
}

}  // namespace alice
}  // namespace isaac

