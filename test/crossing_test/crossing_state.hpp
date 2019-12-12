// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef CROSSINGSTATE_HPP
#define CROSSINGSTATE_HPP

#include <map>
#include <iostream>
#include <random>

#include "ltl_evaluator/evaluator_rule_ltl.h"
#include "test/crossing_test/common.hpp"
#include "ltl_evaluator/evaluator_label_base.h"
#include "test/crossing_test/crossing_state_parameter.h"

using namespace mcts;
using namespace ltl;

namespace mcts {
class Viewer;
}

typedef std::vector<std::multimap<Rule, RuleState>> RuleStateMap;

// A simple environment with a 1D state, only if both agents select different actions, they get nearer to the terminal state
class CrossingState : public mcts::StateInterface<CrossingState> {

 public:
  CrossingState(RuleStateMap rule_state_map, std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
                const CrossingStateParameter &parameters);

  CrossingState(std::vector<AgentState> agent_states,
                const bool terminal,
                RuleStateMap rule_state_map,
                std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
                const CrossingStateParameter &parameters,
                int depth = 0);
  ~CrossingState() {};

  std::shared_ptr<CrossingState> clone() const;

  std::shared_ptr<CrossingState> execute(const JointAction &joint_action, std::vector<Reward> &rewards) const;

  std::vector<Reward> get_final_reward() const;

  template<typename ActionType = Actions>
  ActionType get_last_action(const AgentIdx &agent_idx) const;

  ActionIdx get_num_actions(AgentIdx agent_idx) const;

  bool is_terminal() const;

  const std::vector<AgentIdx> get_agent_idx() const;

  bool ego_goal_reached() const;

  typedef Actions ActionType;

  int get_ego_pos() const;

  const std::vector<AgentState> &get_agent_states() const;

  const RuleStateMap &get_rule_state_map() const;

  void draw(Viewer *viewer) const;

  void reset_depth();

  void update_rule_belief();

  void reset_violations();

  inline int distance_to_ego(const AgentIdx &other_agent_idx) const {
    return agent_states_[ego_agent_idx].x_pos - agent_states_[other_agent_idx + 1].x_pos;
  }

  std::string sprintf() const;
  const CrossingStateParameter &get_parameters() const;

 private:
  Reward get_action_cost(ActionIdx action, AgentIdx agent_idx) const;
  Reward get_shaping_reward(const AgentState &agent_state) const;

  std::vector<AgentState> agent_states_;
  bool terminal_;
  RuleStateMap rule_state_map_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator_;
  int depth_;
  CrossingStateParameter parameters_;
};

#endif
