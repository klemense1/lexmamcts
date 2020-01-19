//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/factories/zipper_test_env_factory.h"
#include "test/crossing_test/state_file_writer.h"

TEST(ZipperMergeTest, violated) {
  auto env = ZipperTestEnvFactory().make_test_env();
  auto agent_states = env->state->get_agent_states();
  agent_states[0].x_pos = env->crossing_state_parameter_.crossing_point - 10;
  agent_states[1].x_pos = env->crossing_state_parameter_.crossing_point - 2;
  agent_states[2].x_pos = env->crossing_state_parameter_.crossing_point - 3;
  agent_states[0].lane = 1;
  agent_states[1].lane = 2;
  agent_states[2].lane = 2;
  std::vector<bool> terminal_agents(agent_states.size(), false);
  env->state = std::make_shared<CrossingState>(
      agent_states, false, env->state->get_rule_state_map(),
      env->label_evaluators_, env->crossing_state_parameter_, 0,
      terminal_agents);
  env->set_jt({0, 0, 0});
  for (size_t i = 0; i < 6; ++i) {
    env->state = env->state->execute(env->get_jt(), env->rewards);
    VLOG(1) << "\n";
    agent_states = env->state->get_agent_states();
    auto others = agent_states;
    others.erase(others.begin() + 2);
    World w(agent_states[2], others);
    for (const auto &label : env->label_evaluators_) {
      auto res = label->evaluate(w);
      for (const auto &le : res) VLOG(1) << le.first << ": " << le.second;
    }
  }
  auto rule_states = env->state->get_rule_state_map();
  EXPECT_TRUE(std::all_of(rule_states.at(0).find(Rule::ZIP),
                          rule_states.at(0).upper_bound(Rule::ZIP),
                          [](const std::pair<Rule, ltl::RuleState> &pair) {
                            return pair.second.get_violation_count() == 0;
                          }));
  EXPECT_TRUE(std::all_of(rule_states.at(1).find(Rule::ZIP),
                          rule_states.at(1).upper_bound(Rule::ZIP),
                          [](const std::pair<Rule, ltl::RuleState> &pair) {
                            return pair.second.get_violation_count() == 0;
                          }));
  EXPECT_TRUE(std::any_of(rule_states.at(2).find(Rule::ZIP),
                          rule_states.at(2).upper_bound(Rule::ZIP),
                          [](const std::pair<Rule, ltl::RuleState> &pair) {
                            return pair.second.get_violation_count() > 0;
                          }));
}

TEST(ZipperMergeTest, not_violated) {
  auto env = ZipperTestEnvFactory().make_test_env();
  auto agent_states = env->state->get_agent_states();
  agent_states[0].x_pos = env->crossing_state_parameter_.crossing_point - 3;
  agent_states[1].x_pos = env->crossing_state_parameter_.crossing_point - 2;
  agent_states[2].x_pos = env->crossing_state_parameter_.crossing_point - 4;
  agent_states[0].lane = 1;
  agent_states[1].lane = 2;
  agent_states[2].lane = 2;
  std::vector<bool> terminal_agents(agent_states.size(), false);
  env->state = std::make_shared<CrossingState>(
      agent_states, false, env->state->get_rule_state_map(),
      env->label_evaluators_, env->crossing_state_parameter_, 0,
      terminal_agents);
  env->set_jt({0, 0, 0});
  std::vector<Eigen::MatrixXi> states;
  for (size_t i = 0; i < 6; ++i) {
    env->state = env->state->execute(env->get_jt(), env->rewards);

    agent_states = env->state->get_agent_states();
    auto others = agent_states;
    others.erase(others.begin());
    World w(agent_states[0], others);
    for (const auto &label : env->label_evaluators_) {
      auto res = label->evaluate(w)[0];
      VLOG(1) << res.first << ": " << res.second;
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
  EXPECT_TRUE(std::all_of(rule_states.at(0).find(Rule::ZIP),
                          rule_states.at(0).upper_bound(Rule::ZIP),
                          [](const std::pair<Rule, ltl::RuleState> &pair) {
                            return pair.second.get_violation_count() == 0;
                          }));
  EXPECT_TRUE(std::all_of(rule_states.at(1).find(Rule::ZIP),
                          rule_states.at(1).upper_bound(Rule::ZIP),
                          [](const std::pair<Rule, ltl::RuleState> &pair) {
                            return pair.second.get_violation_count() == 0;
                          }));
  EXPECT_TRUE(std::any_of(rule_states.at(2).find(Rule::ZIP),
                          rule_states.at(2).upper_bound(Rule::ZIP),
                          [](const std::pair<Rule, ltl::RuleState> &pair) {
                            return pair.second.get_violation_count() == 0;
                          }));
}

TEST(ZipperMergeTest, mcts) {
  auto env = ZipperTestEnvFactory().make_test_env();

  auto agent_states = env->state->get_agent_states();
  std::vector<Eigen::MatrixXi> states;
  Eigen::Vector3i state;
  for (size_t i = 0; i < agent_states.size(); ++i) {
    state(i) = agent_states[i].x_pos;
  }
  char filename[50];
  states.emplace_back(state.transpose());
  int i = 0;
  while (!env->state->is_terminal()) {
    env->search(10000);
    sprintf(filename, "/tmp/zip%d", i);
    std::dynamic_pointer_cast<CrossingTestEnv<mcts::ThresUCTStatistic>>(env)
        ->mcts.printTreeToDotFile(filename);
    env->state = env->state->execute(env->get_jt(), env->rewards);

    agent_states = env->state->get_agent_states();
    auto others = agent_states;
    others.erase(others.begin());
    World w(agent_states[0], others);
    for (const auto &label : env->label_evaluators_) {
      auto res = label->evaluate(w);
      for (const auto &le : res) VLOG(1) << le.first << ": " << le.second;
    }
    auto rule_states = env->state->get_rule_state_map();
    for (size_t i = 0; i < agent_states.size(); ++i) {
      state(i) = agent_states[i].x_pos;
      EXPECT_EQ(
          0, rule_states.at(i).find(Rule::ZIP)->second.get_violation_count());
    }
    VLOG(1) << "Iteration: " << i << ", State: [" << state.transpose() << "]";
    states.emplace_back(state.transpose());
    ++i;
  }
  VLOG(1) << states;
  agent_states = env->state->get_agent_states();
  auto others = agent_states;
  others.erase(others.begin());
  World w(agent_states[0], others);
  for (const auto &label : env->label_evaluators_) {
    auto res = label->evaluate(w);
    for (const auto &le : res) VLOG(1) << le.first << ": " << le.second;
  }
  StateFileWriter sfw(3, "/tmp/xing_zipper_merge.dat");
  sfw.write_multi_timestep(states);
  auto rule_states = env->state->get_rule_state_map();
  for (size_t ai = 0; ai < env->state->get_agent_idx().size(); ++ai) {
    EXPECT_EQ(rule_states.at(ai)
                  .find(Rule::NO_COLLISION)
                  ->second.get_violation_count(),
              0);
    EXPECT_EQ(rule_states.at(ai).find(Rule::ZIP)->second.get_violation_count(),
              0);
    EXPECT_GT(agent_states.at(ai).x_pos,
              env->crossing_state_parameter_.crossing_point);
  }
}

int main(int argc, char **argv) {
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}