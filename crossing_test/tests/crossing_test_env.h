//
// Created by Luis Gressenbuch on 04.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
#define MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_

#include <utility>

#include "crossing_test/tests/base_test_env.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"

template <class Stats = UctStatistic<>, class Heuristic = RandomHeuristic>
class CrossingTestEnv : public BaseTestEnv {
 public:
  explicit CrossingTestEnv(const ObjectiveVec& thres)
      : BaseTestEnv(thres), mcts(mcts_parameters_) {}
  JointAction Search(size_t num_iterations) override {
    mcts.Search(*(this->state), std::numeric_limits<unsigned int>::max(),
                num_iterations);
    this->SetJt(mcts.ReturnBestAction());
    VLOG(1) << "Best action: " << this->GetJt();
    return this->GetJt();
  }
  std::map<unsigned long, Eigen::VectorXf> GetEgoQval() override {
    return mcts.GetRoot()->GetEgoIntNode().GetExpectedRewards();
  }
  Mcts<CrossingState, Stats, Stats, Heuristic> mcts;
};

#endif  // MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
