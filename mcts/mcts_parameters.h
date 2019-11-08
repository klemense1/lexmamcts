// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_PARAMETERS_H
#define MCTS_PARAMETERS_H

#ifndef REWARD_DIM
#define REWARD_DIM 4
#endif

#include "Eigen/Core"

namespace mcts {

typedef Eigen::Matrix<float, REWARD_DIM, 1> ObjectiveVec;

struct MctsParameters {

  struct UctStatistic {
    double EXPLORATION_CONSTANT;
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

  double COOP_FACTOR;
  double DISCOUNT_FACTOR;

  RandomHeuristic random_heuristic;
  UctStatistic uct_statistic;
  EGreedyUCTStatistic e_greedy_uct_statistic_;
  SlackUCTStatistic slack_uct_statistic_;
};

MctsParameters make_std_mcts_parameters();
std::ostream &operator<<(std::ostream &os, MctsParameters const &d);

} // namespace mcts


#endif 
