// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "glog/logging.h"

#include "mcts/mcts_parameters.h"

namespace mcts {

std::ostream &operator<<(std::ostream &os, MctsParameters const &d) {
  LOG(INFO) << "MCTS Parameters:";
  LOG(INFO) << "MctsParameters::MAX_NUMBER_OF_ITERATIONS: "
            << d.random_heuristic.MAX_NUMBER_OF_ITERATIONS;
  LOG(INFO) << "MctsParameters::MAX_SEARCH_TIME_RANDOM_HEURISTIC: " << d.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC;
  LOG(INFO) << "MctsParameters::DISCOUNT_FACTOR: " << d.DISCOUNT_FACTOR;
  LOG(INFO) << "MctsParameters::EXPLORATION_CONSTANT: " << d.uct_statistic.EXPLORATION_CONSTANT;
  LOG(INFO) << "MctsParameters::LOWER_BOUND: " << d.uct_statistic.LOWER_BOUND.transpose();
  LOG(INFO) << "MctsParameters::UPPER_BOUND: " << d.uct_statistic.UPPER_BOUND.transpose();
  LOG(INFO) << "MctsParameters::COOP_FACTOR: " << d.COOP_FACTOR;
  LOG(INFO) << "MctsParameters::EPSILON: " << d.e_greedy_uct_statistic_.EPSILON;
  LOG(INFO) << "MctsParameters::ALPHA: " << d.slack_uct_statistic_.ALPHA;
  return os;
}

} // namespace mcts
