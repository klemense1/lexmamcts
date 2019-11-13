//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "common.h"

#include "glog/logging.h"

std::vector<Reward> get_optimal_reward(std::shared_ptr<CrossingState> const init_state) {
  std::shared_ptr<CrossingState> state = init_state->clone();
  JointAction jt(state->get_agent_idx().size(), aconv(Actions::FORWARD));
  std::vector<Reward> rewards(state->get_agent_idx().size(), Reward::Zero());
  std::vector<Reward> accu_reward(state->get_agent_idx().size(), Reward::Zero());
  DLOG(INFO) << state->sprintf();

  for (int i = 0; i < init_state->get_parameters().crossing_point-2; ++i) {
    state = state->execute(jt, rewards);
    accu_reward += rewards;
    DLOG(INFO) << state->sprintf();
  }

  jt[0] = aconv(Actions::WAIT);
  for (int i = 0; i < 1; ++i) {
    state = state->execute(jt, rewards);
    accu_reward += rewards;
    DLOG(INFO) << state->sprintf();
  }

  jt[0] = aconv(Actions::FORWARD);
  while (!state->is_terminal()) {
    state = state->execute(jt, rewards);
    accu_reward += rewards;
    DLOG(INFO) << state->sprintf();
  }

  accu_reward += state->get_final_reward();
  DLOG(INFO) << "Optimal rewards:";
  DLOG(INFO) << "Ego:" << accu_reward.at(0).transpose();
  for (size_t otr_idx = 1; otr_idx <  state->get_agent_idx().size(); ++otr_idx) {
    DLOG(INFO) << "Agent " << otr_idx << ":" << accu_reward.at(otr_idx).transpose();
  }
  return accu_reward;
}

Eigen::MatrixXf rewards_to_mat(std::vector<Reward> const &rewards) {
  Eigen::MatrixXf mat(Reward::RowsAtCompileTime, rewards.size());
  for (size_t i = 0; i < rewards.size(); ++i) {
    mat.col(i) = rewards[i];
  }
  return mat;
}

MctsParameters make_default_mcts_parameters() {
  MctsParameters param;

  param.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC = 10000;
  param.random_heuristic.MAX_NUMBER_OF_ITERATIONS = 40;
  param.COOP_FACTOR = 0.0;
  param.DISCOUNT_FACTOR = 0.9;

  param.uct_statistic.PROGRESSIVE_WIDENING_ENABLED = false;
  param.uct_statistic.PROGRESSIVE_WIDENING_ALPHA = 0.5;

  param.uct_statistic.EXPLORATION_CONSTANT = 1;
  param.uct_statistic.LOWER_BOUND = Eigen::Vector4f(-80000.0f, -1000.0f, -1000.0f, -1000.0f);
  param.uct_statistic.UPPER_BOUND = Eigen::Vector4f(0.0f, 1000.0f, 0.0f, 0.0f);

  param.e_greedy_uct_statistic_.EPSILON = 0.1;

  param.slack_uct_statistic_.ALPHA = 0.05;

  param.thres_uct_statistic_.THRESHOLD << -500, 0, 0, 0;

  return param;
}