//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_UTIL_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_UTIL_H_

#include <vector>
#include <memory>

#include "Eigen/Core"

#include "test/crossing_test/crossing_state.hpp"
#include "mcts/mcts.h"

template <class TestEnv>
void get_optimal_reward(TestEnv* t_env) {
  JointAction jt(t_env->state->get_agent_idx().size(), aconv(Actions::FORWARD));
  std::vector<Reward> rewards(t_env->state->get_agent_idx().size(), Reward::Zero());
  std::vector<Reward> accu_reward(t_env->state->get_agent_idx().size(), Reward::Zero());
  DLOG(INFO) << t_env->state->sprintf();

  for (int i = 0; i < t_env->state->get_parameters().crossing_point-2; ++i) {
    t_env->set_jt(jt);
    t_env->state = t_env->state->execute(jt, rewards);
    accu_reward += rewards;
    DLOG(INFO) << t_env->state->sprintf();
  }

  jt[0] = aconv(Actions::WAIT);
  for (int i = 0; i < 1; ++i) {
    t_env->set_jt(jt);
    t_env->state = t_env->state->execute(jt, rewards);
    accu_reward += rewards;
    DLOG(INFO) << t_env->state->sprintf();
  }

  jt[0] = aconv(Actions::FORWARD);
  while (!t_env->state->is_terminal()) {
    t_env->set_jt(jt);
    t_env->state = t_env->state->execute(jt, rewards);
    accu_reward += rewards;
    DLOG(INFO) << t_env->state->sprintf();
  }

  accu_reward += t_env->state->get_final_reward();
  DLOG(INFO) << "Optimal rewards:";
  DLOG(INFO) << "Ego:" << accu_reward.at(0).transpose();
  for (size_t otr_idx = 1; otr_idx <  t_env->state->get_agent_idx().size(); ++otr_idx) {
    DLOG(INFO) << "Agent " << otr_idx << ":" << accu_reward.at(otr_idx).transpose();
  }
  t_env->rewards = accu_reward;
}
Eigen::MatrixXf rewards_to_mat(std::vector<Reward> const &rewards);
MctsParameters make_default_mcts_parameters();

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_UTIL_H_
