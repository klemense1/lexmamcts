//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_TESTS_BASE_TEST_ENV_H_
#define TEST_CROSSING_TEST_TESTS_BASE_TEST_ENV_H_

#include <deque>
#include <map>
#include <memory>
#include <vector>

#include "mcts/mcts.h"
#include "mcts/random_generator.h"
#include "mcts/statistics/slack_uct_statistic.h"
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_at_position.h"
#include "test/crossing_test/label_evaluator/evaluator_label_collision.h"
#include "test/crossing_test/tests/util.h"

using namespace mcts;
using RuleMonitorSPtr = RuleMonitor::RuleMonitorSPtr;

class BaseTestEnv {
 public:
  explicit BaseTestEnv(
      MctsParameters mcts_parameters = MakeDefaultMctsParameters(),
      CrossingStateParameter crossing_state_parameter =
          MakeDefaultCrossingStateParameters(),
      std::vector<std::map<Rule, RuleMonitorSPtr>> automata =
          MakeDefaultAutomata(
              MakeDefaultCrossingStateParameters().num_other_agents + 1),
      std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators =
          MakeDefaultLabels(MakeDefaultCrossingStateParameters()));
  ~BaseTestEnv();

  static std::vector<std::map<Rule, RuleMonitorSPtr>> MakeDefaultAutomata(size_t num_agents);

  static std::vector<std::shared_ptr<EvaluatorLabelBase<World>>>
  MakeDefaultLabels(
      const CrossingStateParameter &params);

  virtual JointAction Search(size_t num_iterations) = 0;

  virtual std::map<unsigned long, Eigen::VectorXf> GetEgoQval() = 0;
  const JointAction &GetJt() const;
  void SetJt(const JointAction &jt);
  const std::deque<JointAction> &GetActionHistory() const;

  const MctsParameters mcts_parameters_;
  const CrossingStateParameter crossing_state_parameter_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators_;
  std::vector<Reward> rewards;
  std::vector<Eigen::MatrixXi> state_history_;
  std::shared_ptr<CrossingState> state;

 protected:
  std::vector<std::map<Rule, RuleMonitorSPtr>> automata_;

  RuleStateMap GetAutomataVec() const;

 private:
  void CreateState();

  JointAction jt_;
  std::deque<JointAction> action_history_;
};

#endif  // TEST_CROSSING_TEST_TESTS_BASE_TEST_ENV_H_
