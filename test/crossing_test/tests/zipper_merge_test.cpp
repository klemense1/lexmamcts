//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "gflags/gflags.h"

#include "test/crossing_test/common.hpp"
#include "zipper_test_env_factory.h"

TEST(ZipperMergeTest, violated) {
  auto env = ZipperTestEnvFactory().make_test_env();
  auto agent_states = env->state->get_agent_states();
  agent_states[0].x_pos = env->crossing_state_parameter_.crossing_point - 5;
  agent_states[1].x_pos = env->crossing_state_parameter_.crossing_point - 1;
  agent_states[2].x_pos = env->crossing_state_parameter_.crossing_point - 3;
  agent_states[2].lane = agent_states[1].lane;
  env->state = std::make_shared<CrossingState>(
      agent_states, false, env->state->get_rule_state_map(),
      env->label_evaluators_, env->crossing_state_parameter_, 0);
  env->set_jt({0,0,0});
  for(size_t i = 0; i < 4; ++i) {
    env->state = env->state->execute(env->get_jt(), env->rewards);

    agent_states = env->state->get_agent_states();
    auto others = agent_states;
    others.erase(others.begin());
    World w(agent_states[0], others);
    for (const auto &label : env->label_evaluators_) {
      VLOG(1) << label->get_label() << ": " << label->evaluate(w);
    }

  }
  auto rule_states = env->state->get_rule_state_map();
  EXPECT_EQ(rule_states.at(0).find(Rule::ZIP)->second.get_violation_count(), 0);
  EXPECT_EQ(rule_states.at(1).find(Rule::ZIP)->second.get_violation_count(), 0);
  EXPECT_GT(rule_states.at(2).find(Rule::ZIP)->second.get_violation_count(), 0);
}

TEST(ZipperMergeTest, not_violated) {
  auto env = ZipperTestEnvFactory().make_test_env();
  auto agent_states = env->state->get_agent_states();
  agent_states[0].x_pos = env->crossing_state_parameter_.crossing_point - 3;
  agent_states[1].x_pos = env->crossing_state_parameter_.crossing_point - 1;
  agent_states[2].x_pos = env->crossing_state_parameter_.crossing_point - 5;
  agent_states[2].lane = agent_states[1].lane;
  env->state = std::make_shared<CrossingState>(
      agent_states, false, env->state->get_rule_state_map(),
      env->label_evaluators_, env->crossing_state_parameter_, 0);
  env->set_jt({0,0,0});
  std::vector<Eigen::MatrixXi> states;
  for(size_t i = 0; i < 6; ++i) {
    env->state = env->state->execute(env->get_jt(), env->rewards);

    agent_states = env->state->get_agent_states();
    auto others = agent_states;
    others.erase(others.begin());
    World w(agent_states[0], others);
    for (const auto &label : env->label_evaluators_) {
      VLOG(1) << label->get_label() << ": " << label->evaluate(w);
    }

    Eigen::Vector3i state;
    for (size_t i = 0; i < agent_states.size(); ++i) {
      state(i) = agent_states[i].x_pos;
    }
    VLOG(1) << state.transpose();
    states.emplace_back(state.transpose());

  }
  VLOG(1) << states;
  auto rule_states = env->state->get_rule_state_map();
  EXPECT_EQ(rule_states.at(0).find(Rule::ZIP)->second.get_violation_count(), 0);
  EXPECT_EQ(rule_states.at(1).find(Rule::ZIP)->second.get_violation_count(), 0);
  EXPECT_EQ(rule_states.at(2).find(Rule::ZIP)->second.get_violation_count(), 0);
}


TEST(ZipperMergeTest, mcts) {
  auto env = ZipperTestEnvFactory().make_test_env();

  auto agent_states = env->state->get_agent_states();
  std::vector<Eigen::MatrixXi> states;
  Eigen::Vector3i state;
  for (size_t i = 0; i < agent_states.size(); ++i) {
    state(i) = agent_states[i].x_pos;
  }

  states.emplace_back(state.transpose());
  while (!env->state->is_terminal()) {
    env->search(10000);
    env->state = env->state->execute(env->get_jt(), env->rewards);

    agent_states = env->state->get_agent_states();
    auto others = agent_states;
    others.erase(others.begin());
    World w(agent_states[0], others);
    for (const auto &label : env->label_evaluators_) {
      VLOG(1) << label->get_label() << ": " << label->evaluate(w);
    }
    auto rule_states = env->state->get_rule_state_map();
    for (size_t i = 0; i < agent_states.size(); ++i) {
      state(i) = agent_states[i].x_pos;
      ASSERT_EQ(rule_states.at(i).find(Rule::ZIP)->second.get_violation_count(), 0);
    }
    VLOG(1) << state.transpose();
    states.emplace_back(state.transpose());
  }
  VLOG(1) << states;
  auto rule_states = env->state->get_rule_state_map();
  for(size_t ai = 0; ai < env->state->get_agent_idx().size(); ++ai) {
    EXPECT_EQ(rule_states.at(ai).find(Rule::NO_COLLISION)->second.get_violation_count(), 0);
    EXPECT_EQ(rule_states.at(ai).find(Rule::ZIP)->second.get_violation_count(), 0);
    EXPECT_GT(agent_states.at(ai).x_pos, env->crossing_state_parameter_.crossing_point);
  }
}

int main(int argc, char **argv) {
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}