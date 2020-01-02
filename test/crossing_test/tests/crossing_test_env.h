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
      MctsParameters mcts_parameters = make_default_mcts_parameters(),
      CrossingStateParameter crossing_state_parameter = make_default_crossing_state_parameters(),
                  std::vector<std::map<Rule, EvaluatorRuleLTLSPtr>> automata = BaseTestEnv::make_default_automata(
                      make_default_crossing_state_parameters().num_other_agents + 1),
                  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators = BaseTestEnv::make_default_labels(
                      make_default_crossing_state_parameters())) : BaseTestEnv(mcts_parameters,
                                                                               crossing_state_parameter,
                                                                               automata,
                                                                               label_evaluators),
                                                                   mcts(mcts_parameters_) {}
  JointAction search(size_t num_iterations) override {
    mcts.search(*(this->state), 50000, num_iterations);
    this->set_jt(mcts.returnBestAction());
    VLOG(1) << "Best action: " << this->get_jt();
    return this->get_jt();
  }
  Mcts<CrossingState, Stats, Stats, Heuristic> mcts;
};

#endif  // MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
