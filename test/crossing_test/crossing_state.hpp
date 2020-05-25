// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef TEST_CROSSING_TEST_CROSSING_STATE_HPP_
#define TEST_CROSSING_TEST_CROSSING_STATE_HPP_

#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "label_evaluator/evaluator_label_base.h"
#include "ltl/rule_monitor.h"
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/crossing_state_parameter.h"

using namespace mcts;
using namespace ltl;

namespace mcts {
class Viewer;
}

typedef std::vector<std::multimap<Rule, RuleState>> RuleStateMap;

// A simple environment with a 1D state, only if both agents select different
// actions, they get nearer to the terminal state
class CrossingState : public mcts::StateInterface<CrossingState> {
 public:
  typedef Actions ActionType;

  CrossingState(
      RuleStateMap rule_state_map,
      std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
      const CrossingStateParameter &parameters);

  CrossingState(
      std::vector<AgentState> agent_states, const bool terminal,
      RuleStateMap rule_state_map,
      std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
      const CrossingStateParameter &parameters, int depth,
      const std::vector<bool> &terminal_agents);
  ~CrossingState() = default;

  std::shared_ptr<CrossingState> Clone() const;

  std::shared_ptr<CrossingState> Execute(const JointAction &joint_action,
                                         std::vector<Reward> &rewards) const;

  std::vector<Reward> GetTerminalReward() const;

  template <typename ActionType = Actions>
  ActionType GetLastAction(const AgentIdx &agent_idx) const;

  ActionIdx GetNumActions(AgentIdx agent_idx) const;

  bool IsTerminal() const;

  const std::vector<AgentIdx> GetAgentIdx() const;

  bool EgoGoalReached() const;

  int GetEgoPos() const;

  const std::vector<AgentState> &GetAgentStates() const;

  const RuleStateMap &GetRuleStateMap() const;

  void ResetDepth();

  std::string PrintState() const;

  const CrossingStateParameter &GetParameters() const;

  EvaluationMap GetAgentLabels(AgentIdx agent_idx) const;

 private:
  std::vector<AgentState> Step(const JointAction &joint_action) const;
  Reward GetActionCost(ActionIdx action, AgentIdx agent_idx) const;
  Reward GetShapingReward(const AgentState &agent_state) const;
  void FixCollisionPositions(std::vector<AgentState> *agent_states) const;
  std::vector<AgentState> agent_states_;
  bool terminal_;
  RuleStateMap rule_state_map_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator_;
  int depth_;
  CrossingStateParameter parameters_;
  std::vector<bool> terminal_agents_;
};

#endif  // TEST_CROSSING_TEST_CROSSING_STATE_HPP_
