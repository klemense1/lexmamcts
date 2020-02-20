//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "util.h"

Eigen::MatrixXf rewards_to_mat(std::vector<Reward> const &rewards) {
  Eigen::MatrixXf mat(rewards[0].rows(), rewards.size());
  for (size_t i = 0; i < rewards.size(); ++i) {
    mat.col(i) = rewards[i];
  }
  return mat;
}

MctsParameters make_default_mcts_parameters() {
  MctsParameters param;

  param.REWARD_VEC_SIZE = 3;

  param.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC = std::numeric_limits<double>::max();
  param.random_heuristic.MAX_NUMBER_OF_ITERATIONS = 30;
  param.COOP_FACTOR = 0.0;
  param.DISCOUNT_FACTOR = 0.9;

  param.uct_statistic.PROGRESSIVE_WIDENING_ENABLED = false;
  param.uct_statistic.PROGRESSIVE_WIDENING_ALPHA = 0.5;

  param.uct_statistic.EXPLORATION_CONSTANT = 0.7;
  param.uct_statistic.LOWER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);
  param.uct_statistic.LOWER_BOUND << -1.0f, -1.0f, -5000.0f;
  param.uct_statistic.UPPER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);
  param.uct_statistic.UPPER_BOUND << 0.0f, 0.0f, 0.0f;

  param.e_greedy_uct_statistic_.EPSILON = 0.2;

  param.slack_uct_statistic_.ALPHA = 0.05;

  param.thres_uct_statistic_.THRESHOLD = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);
  param.thres_uct_statistic_.THRESHOLD << -0.28, -0.44, std::numeric_limits<ObjectiveVec::Scalar>::max();

  return param;
}