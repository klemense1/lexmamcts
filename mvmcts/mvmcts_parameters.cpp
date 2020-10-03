// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "glog/logging.h"

#include "mvmcts/mvmcts_parameters.h"

namespace mvmcts {

std::ostream& operator<<(std::ostream& os, const MvmctsParameters& parameters) {
  os << "REWARD_VEC_SIZE: " << parameters.REWARD_VEC_SIZE
     << " COOP_FACTOR: " << parameters.COOP_FACTOR
     << " DISCOUNT_FACTOR: " << parameters.DISCOUNT_FACTOR
     << " random_heuristic: " << parameters.random_heuristic
     << " uct_statistic: " << parameters.uct_statistic
     << " thres_greedy_statistic_: " << parameters.thres_greedy_statistic_
     << " slack_uct_statistic_: " << parameters.slack_uct_statistic_
     << " thres_uct_statistic_: " << parameters.thres_uct_statistic_;
  return os;
}
std::ostream& operator<<(std::ostream& os,
                         const MvmctsParameters::UctStatistic& statistic) {
  os << "EXPLORATION_CONSTANT: " << statistic.EXPLORATION_CONSTANT
     << " LOWER_BOUND: " << statistic.LOWER_BOUND
     << " UPPER_BOUND: " << statistic.UPPER_BOUND;
  return os;
}
std::ostream& operator<<(std::ostream& os,
                         const MvmctsParameters::RandomHeuristic& heuristic) {
  os << "MAX_NUMBER_OF_ITERATIONS: " << heuristic.MAX_NUMBER_OF_ITERATIONS
     << " MAX_SEARCH_TIME_RANDOM_HEURISTIC: "
     << heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC;
  return os;
}
std::ostream& operator<<(
    std::ostream& os, const MvmctsParameters::ThresGreedyStatistic& statistic) {
  os << "DECAY1: " << statistic.DECAY1 << " DECAY2: " << statistic.DECAY2;
  return os;
}
std::ostream& operator<<(std::ostream& os,
                         const MvmctsParameters::SlackUCTStatistic& statistic) {
  os << "SLACK_FACTOR: " << statistic.SLACK_FACTOR;
  return os;
}
std::ostream& operator<<(std::ostream& os,
                         const MvmctsParameters::ThresUCTStatistic& statistic) {
  os << "THRESHOLD: " << statistic.THRESHOLD
     << " EPSILON: " << statistic.EPSILON;
  return os;
}
}  // namespace mvmcts
