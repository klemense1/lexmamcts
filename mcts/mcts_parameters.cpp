// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "glog/logging.h"

#include "mcts/mcts_parameters.h"

namespace mcts {

MctsParameters make_std_mcts_parameters() {
  MctsParameters param;

  param.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC = 1000;
  param.random_heuristic.MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC = 1000;
  param.COOP_FACTOR = 0;
  param.DISCOUNT_FACTOR = 1;

  param.uct_statistic.EXPLORATION_CONSTANT = 1;
  param.uct_statistic.LOWER_BOUND = Eigen::Vector4f(-2000.0f, -1000.0f, -1000.0f, -1000.0f);
  param.uct_statistic.UPPER_BOUND = Eigen::Vector4f(2000.0f, 0.0f, 0.0f, 0.0f);

  return param;
}

std::ostream &operator<<(std::ostream &os, MctsParameters const &d) {
  LOG(INFO) << "MCTS Parameters:";
  LOG(INFO) << "MctsParameters::MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC: "
            << d.random_heuristic.MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC;
  LOG(INFO) << "MctsParameters::MAX_SEARCH_TIME_RANDOM_HEURISTIC: " << d.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC;
  LOG(INFO) << "MctsParameters::DISCOUNT_FACTOR: " << d.DISCOUNT_FACTOR;
  LOG(INFO) << "MctsParameters::EXPLORATION_CONSTANT: " << d.uct_statistic.EXPLORATION_CONSTANT;
  LOG(INFO) << "MctsParameters::LOWER_BOUND: " << d.uct_statistic.LOWER_BOUND.transpose();
  LOG(INFO) << "MctsParameters::UPPER_BOUND: " << d.uct_statistic.UPPER_BOUND.transpose();
  LOG(INFO) << "MctsParameters::COOP_FACTOR: " << d.COOP_FACTOR;
  return os;
}

} // namespace mcts
