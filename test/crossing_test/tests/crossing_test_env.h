//
// Created by Luis Gressenbuch on 04.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
#define MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_

#include <utility>

#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "test/crossing_test/tests/base_test_env.h"

template<class Stats = UctStatistic<>, class Heuristic = RandomHeuristic>
class CrossingTestEnv : public BaseTestEnv {
 public:
  explicit CrossingTestEnv(
      MctsParameters mcts_parameters = MakeDefaultMctsParameters(),
      CrossingStateParameter crossing_state_parameter =
          MakeDefaultCrossingStateParameters(),
      std::vector<std::map<Rule, RuleMonitorSPtr>> automata =
          BaseTestEnv::MakeDefaultAutomata(
              MakeDefaultCrossingStateParameters().num_other_agents + 1),
                  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators =
          BaseTestEnv::MakeDefaultLabels(MakeDefaultCrossingStateParameters())) : BaseTestEnv(mcts_parameters,
                                                                               crossing_state_parameter,
                                                                               automata,
                                                                               label_evaluators),
                                                                   mcts(mcts_parameters_) {}
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
