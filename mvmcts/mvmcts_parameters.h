// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_MVMCTS_PARAMETERS_H_
#define MVMCTS_MVMCTS_PARAMETERS_H_

#include <ostream>
#include "Eigen/Core"

namespace mvmcts {

typedef Eigen::VectorXf ObjectiveVec;

struct MvmctsParameters {
  friend std::ostream& operator<<(std::ostream& os,
                                  const MvmctsParameters& parameters);
  struct UctStatistic {
    friend std::ostream& operator<<(std::ostream& os,
                                    const UctStatistic& statistic);
    Eigen::VectorXd EXPLORATION_CONSTANT;
    ObjectiveVec LOWER_BOUND;
    ObjectiveVec UPPER_BOUND;
  };

  struct RandomHeuristic {
    friend std::ostream& operator<<(std::ostream& os,
                                    const RandomHeuristic& heuristic);
    int MAX_NUMBER_OF_ITERATIONS;
    double MAX_SEARCH_TIME_RANDOM_HEURISTIC;
  };

  struct ThresGreedyStatistic {
    friend std::ostream& operator<<(std::ostream& os,
                                    const ThresGreedyStatistic& statistic);
    double DECAY1;
    double DECAY2;
  };

  struct SlackUCTStatistic {
    friend std::ostream& operator<<(std::ostream& os,
                                    const SlackUCTStatistic& statistic);
    float SLACK_FACTOR;
  };

  struct ThresUCTStatistic {
    friend std::ostream& operator<<(std::ostream& os,
                                    const ThresUCTStatistic& statistic);
    ObjectiveVec THRESHOLD;
    double EPSILON;
  };

  size_t REWARD_VEC_SIZE = 5;

  double COOP_FACTOR;
  double DISCOUNT_FACTOR;

  RandomHeuristic random_heuristic;
  UctStatistic uct_statistic;
  ThresGreedyStatistic thres_greedy_statistic_;
  SlackUCTStatistic slack_uct_statistic_;
  ThresUCTStatistic thres_uct_statistic_;
};

}  // namespace mvmcts

#endif  // MVMCTS_MVMCTS_PARAMETERS_H_
