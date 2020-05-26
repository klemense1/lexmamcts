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

#include "gridworld/common.hpp"
#include "gridworld/grid_world_state.hpp"
#include "gridworld/label_evaluator/evaluator_label_at_position.h"
#include "gridworld/label_evaluator/evaluator_label_collision.h"
#include "mcts/mcts.h"
#include "mcts/random_generator.h"
#include "mcts/statistics/slack_uct_statistic.h"

using namespace mcts;
using RuleMonitorSPtr = RuleMonitor::RuleMonitorSPtr;

class BaseTestEnv {
 public:
  explicit BaseTestEnv(const ObjectiveVec &thres);
  ~BaseTestEnv() = default;
  static MctsParameters MakeMctsParameters();
  static GridWorldStateParameter MakeGridWorldStateParameters();
  static std::vector<std::map<Rule, RuleMonitorSPtr>> MakeAutomata(
      size_t num_agents);
  static std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> MakeLabels(const GridWorldStateParameter &params);

  virtual JointAction Search(size_t num_iterations) = 0;
  virtual std::map<unsigned long, Eigen::VectorXf> GetEgoQval() = 0;

  const JointAction &GetJt() const;
  const std::deque<JointAction> &GetActionHistory() const;
  void SetJt(const JointAction &jt);

  const MctsParameters mcts_parameters_;
  const GridWorldStateParameter grid_world_state_parameter_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators_;
  std::vector<Reward> rewards;
  std::vector<Eigen::MatrixXi> state_history_;
  std::shared_ptr<GridWorldState> state;

 protected:
  RuleStateMap GetAutomataVec() const;

  std::vector<std::map<Rule, RuleMonitorSPtr>> automata_;

 private:
  JointAction jt_;
  std::deque<JointAction> action_history_;
};

#endif  // TEST_CROSSING_TEST_TESTS_BASE_TEST_ENV_H_
