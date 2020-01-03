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
#include "mcts/statistics/pareto_uct_statistic.h"
#include "mcts/statistics/slack_uct_statistic.h"
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_at_position.h"
#include "test/crossing_test/label_evaluator/evaluator_label_collision.h"
#include "test/crossing_test/label_evaluator/evaluator_label_goal_reached.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_goal_reached.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_near.h"
#include "test/crossing_test/label_evaluator/evaluator_label_speed.h"
#include "test/crossing_test/tests/util.h"
#include "test/crossing_test/viewer.h"

using namespace mcts;
using RuleMonitorSPtr = RuleMonitor::RuleMonitorSPtr;

class BaseTestEnv {
 public:
  explicit BaseTestEnv(
      MctsParameters mcts_parameters = make_default_mcts_parameters(),
      CrossingStateParameter crossing_state_parameter =
          make_default_crossing_state_parameters(),
      std::vector<std::map<Rule, RuleMonitorSPtr>> automata =
          make_default_automata(
              make_default_crossing_state_parameters().num_other_agents + 1),
      std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators =
          make_default_labels(make_default_crossing_state_parameters()));
  ~BaseTestEnv();

  static std::vector<std::map<Rule, RuleMonitorSPtr>> make_default_automata(size_t num_agents);

  static std::vector<std::shared_ptr<EvaluatorLabelBase<World>>>
  make_default_labels(CrossingStateParameter params);

  virtual JointAction search(size_t num_iterations) = 0;

  const JointAction &get_jt() const;
  void set_jt(const JointAction &jt);
  const std::deque<JointAction> &get_action_history() const;

  const MctsParameters mcts_parameters_;
  const CrossingStateParameter crossing_state_parameter_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators_;
  std::vector<Reward> rewards;
  std::vector<std::size_t> pos_history;
  std::vector<size_t> pos_history_other;
  std::shared_ptr<CrossingState> state;

 protected:
  std::vector<std::map<Rule, RuleMonitorSPtr>> automata_;

  RuleStateMap get_automata_vec() const;

 private:
  void create_state();

  JointAction jt;
  std::deque<JointAction> action_history;
};

#endif  // TEST_CROSSING_TEST_TESTS_BASE_TEST_ENV_H_
