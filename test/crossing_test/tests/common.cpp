//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "common.h"

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
  param.uct_statistic.LOWER_BOUND = Eigen::Vector4f(-5000.0f, -5000.0f, -5000.0f, -5000.0f);
  param.uct_statistic.UPPER_BOUND = Eigen::Vector4f(0.0f, 1000.0f, 0.0f, 5000.0f);

  param.e_greedy_uct_statistic_.EPSILON = 0.1;

  param.slack_uct_statistic_.ALPHA = 0.05;

  param.thres_uct_statistic_.THRESHOLD << -0.44, -0.541, -0.99, std::numeric_limits<float>::max();

  return param;
}