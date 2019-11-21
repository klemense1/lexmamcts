// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_PARAMETERS_H
#define MCTS_PARAMETERS_H

#include "Eigen/Core"

namespace mcts {

typedef Eigen::VectorXf ObjectiveVec;

struct MctsParameters {

  struct UctStatistic {
    double EXPLORATION_CONSTANT;
    bool PROGRESSIVE_WIDENING_ENABLED;
    double PROGRESSIVE_WIDENING_ALPHA;
    ObjectiveVec LOWER_BOUND;
    ObjectiveVec UPPER_BOUND;
  };

  struct RandomHeuristic {
    int MAX_NUMBER_OF_ITERATIONS;
    double MAX_SEARCH_TIME_RANDOM_HEURISTIC;
  };

  struct EGreedyUCTStatistic {
    /// Probability of random exploration
    double EPSILON;
  };

  struct SlackUCTStatistic {
    /// Confidence level 1-ALPHA
    double ALPHA;
  };

  struct ThresUCTStatistic {
    ObjectiveVec THRESHOLD;
  };

  size_t REWARD_VEC_SIZE = 5;

  double COOP_FACTOR;
  double DISCOUNT_FACTOR;

  RandomHeuristic random_heuristic;
  UctStatistic uct_statistic;
  EGreedyUCTStatistic e_greedy_uct_statistic_;
  SlackUCTStatistic slack_uct_statistic_;
  ThresUCTStatistic thres_uct_statistic_;
};

std::ostream &operator<<(std::ostream &os, MctsParameters const &d);

} // namespace mcts


#endif 
