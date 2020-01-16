//
// Created by Luis Gressenbuch on 12.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "gflags/gflags.h"

#include "test/crossing_test/common.hpp"
#include "mcts/mcts_parameters.h"
#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/crossing_state_parameter.h"
#include "test/crossing_test/tests/crossing_test_env.h"

TEST(ScalarVsLex, scalar) {
  MctsParameters mcts_params = make_default_mcts_parameters();
  mcts_params.REWARD_VEC_SIZE = 1;
  mcts_params.uct_statistic.LOWER_BOUND = ObjectiveVec::Zero(1);
  mcts_params.uct_statistic.LOWER_BOUND << -5000.0f;
  mcts_params.uct_statistic.UPPER_BOUND = ObjectiveVec::Zero(1);
  mcts_params.uct_statistic.UPPER_BOUND << 5000.0f;

  CrossingStateParameter crossing_params =
      make_default_crossing_state_parameters();
  crossing_params.reward_vec_size = 1;
  crossing_params.depth_prio = 0;
  crossing_params.speed_deviation_prio = 0;
  crossing_params.acceleration_prio = 0;
  crossing_params.potential_prio = 0;
  crossing_params.depth_weight = 0;
  crossing_params.speed_deviation_weight = 0;
  crossing_params.acceleration_weight = 0;
  crossing_params.potential_weight = 0;
  crossing_params.num_other_agents = 0;
  crossing_params.ego_goal_reached_position = 2;
  crossing_params.state_x_length = 3;
  crossing_params.crossing_point = 1;

  auto automata =
      BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
  for (auto &aut : automata) {
    aut.insert({Rule::ZIP, RuleMonitor::make_rule("G !at_xing", -10, 0)});
    aut.at(Rule::REACH_GOAL)->set_weight(-1.0f);
    aut.at(Rule::REACH_GOAL)->set_final_reward(0);
    aut.at(Rule::REACH_GOAL)->set_priority(0);
    aut.erase(Rule::NO_COLLISION);
    aut.erase(Rule::NO_SPEEDING);
    aut.erase(Rule::GIVE_WAY);
    aut.erase(Rule::LEAVE_INTERSECTION);
  }
  auto env = std::make_shared<CrossingTestEnv<UctStatistic<>>>(
      mcts_params, crossing_params, automata,
          BaseTestEnv::make_default_labels(crossing_params));
  auto agent_states = env->state->get_agent_states();
  //agent_states[0].x_pos = crossing_params.crossing_point - 10;

  std::vector<bool> terminal_agents(crossing_params.num_other_agents + 1,
                                    false);
  env->state = std::make_shared<CrossingState>(
      agent_states, false, env->state->get_rule_state_map(),
      env->label_evaluators_, env->crossing_state_parameter_, 0,
      terminal_agents);

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
    }
    VLOG(1) << state.transpose();
    states.emplace_back(state.transpose());
  }
  VLOG(1) << states;
  auto rule_states = env->state->get_rule_state_map();
}

TEST(ScalarVsLex, lex) {
  MctsParameters mcts_params = make_default_mcts_parameters();
  mcts_params.REWARD_VEC_SIZE = 2;
  mcts_params.uct_statistic.LOWER_BOUND = ObjectiveVec::Constant(mcts_params.REWARD_VEC_SIZE, -5000.0f);
  mcts_params.uct_statistic.UPPER_BOUND = ObjectiveVec::Constant(mcts_params.REWARD_VEC_SIZE, 5000.0f);
  mcts_params.thres_uct_statistic_.THRESHOLD = ObjectiveVec::Zero(mcts_params.REWARD_VEC_SIZE);
  mcts_params.thres_uct_statistic_.THRESHOLD << -1.0, std::numeric_limits<ObjectiveVec::Scalar>::infinity();

  CrossingStateParameter crossing_params =
      make_default_crossing_state_parameters();
  crossing_params.reward_vec_size = mcts_params.REWARD_VEC_SIZE;
  crossing_params.depth_prio = 0;
  crossing_params.speed_deviation_prio = 0;
  crossing_params.acceleration_prio = 0;
  crossing_params.potential_prio = 1;
  crossing_params.depth_weight = 0;
  crossing_params.speed_deviation_weight = 0;
  crossing_params.acceleration_weight = 0;
  crossing_params.potential_weight = 0;
  crossing_params.num_other_agents = 0;
  crossing_params.ego_goal_reached_position = 2;
  crossing_params.state_x_length = 3;
  crossing_params.crossing_point = 1;

  auto automata =
      BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
  for (auto &aut : automata) {
    aut.insert({Rule::ZIP, RuleMonitor::make_rule("G !at_xing", -1.0f, 0)});
    aut.at(Rule::REACH_GOAL)->set_weight(-1.0f);
    aut.at(Rule::REACH_GOAL)->set_final_reward(0.0f);
    aut.at(Rule::REACH_GOAL)->set_priority(1);
    aut.erase(Rule::NO_COLLISION);
    aut.erase(Rule::NO_SPEEDING);
    aut.erase(Rule::GIVE_WAY);
    aut.erase(Rule::LEAVE_INTERSECTION);
  }
  auto env = std::make_shared<CrossingTestEnv<ThresUCTStatistic>>(
      mcts_params, crossing_params, automata,
      BaseTestEnv::make_default_labels(crossing_params));
  auto agent_states = env->state->get_agent_states();
  //agent_states[0].x_pos = crossing_params.crossing_point - 10;

  std::vector<bool> terminal_agents(crossing_params.num_other_agents + 1,
                                    false);
  env->state = std::make_shared<CrossingState>(
      agent_states, false, env->state->get_rule_state_map(),
      env->label_evaluators_, env->crossing_state_parameter_, 0,
      terminal_agents);

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
    }
    VLOG(1) << state.transpose();
    states.emplace_back(state.transpose());
  }
  VLOG(1) << states;
  EXPECT_LT( env->state->get_agent_states()[0].x_pos, crossing_params.crossing_point);
  auto rule_states = env->state->get_rule_state_map();
}

int main(int argc, char **argv) {
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv, false);
  FLAGS_v = 1;
  FLAGS_logtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}