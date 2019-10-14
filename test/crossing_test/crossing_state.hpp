// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef SIMPLESTATE_H
#define SIMPLESTATE_H

#include <map>
#include <iostream>
#include <random>

#include "evaluator_rule_ltl.hpp"
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/evaluator_label_base.hpp"

using namespace mcts;
using namespace modules::models::behavior;


// A simple environment with a 1D state, only if both agents select different actions, they get nearer to the terminal state
class CrossingState : public mcts::StateInterface<CrossingState> {
 private:
  static const unsigned int num_other_agents = 1;

 public:

  static const int state_x_length = 21;
  static const int ego_goal_reached_position = 19;
  static const int crossing_point = (state_x_length - 1) / 2 + 1;

  CrossingState(const std::vector<EvaluatorRuleLTL> &automata,
                const std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator) :
      other_agent_states_(num_other_agents),
      ego_state_(),
      terminal_(false),
      automata_(automata),
      label_evaluator_(label_evaluator),
      time_penalty_(0.0f) {
    for (auto &state : other_agent_states_) {
      state = AgentState();
    }
  }

  CrossingState(const std::vector<AgentState> &other_agent_states,
                const AgentState &ego_state,
                const bool &terminal,
                const std::vector<EvaluatorRuleLTL> &automata,
                const std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
                const float time_penalty) :
      other_agent_states_(other_agent_states),
      ego_state_(ego_state),
      terminal_(terminal),
      automata_(automata),
      label_evaluator_(label_evaluator),
      time_penalty_(time_penalty) {};
  ~CrossingState() {};

  std::shared_ptr<CrossingState> clone() const {
    return std::make_shared<CrossingState>(*this);
  }

  template<typename ActionType = Actions>
  ActionType get_last_action(const AgentIdx &agent_idx) const {
    if (agent_idx == ego_agent_idx) {
      return ego_state_.last_action;
    } else {
      return other_agent_states_[agent_idx - 1].last_action;
    }
  }

  std::shared_ptr<CrossingState> execute(const JointAction &joint_action, std::vector<Reward> &rewards) const {
    // normally we map each single action value in joint action with a map to the floating point action. Here, not required
    int new_x_ego = ego_state_.x_pos + static_cast<int>(aconv(joint_action[ego_agent_idx]));
    bool ego_out_of_map = false;
    if (new_x_ego < 0) {
      ego_out_of_map = true;
    }
    const AgentState next_ego_state
        (ego_state_.x_pos + static_cast<int>(aconv(joint_action[ego_agent_idx])), aconv(joint_action[ego_agent_idx]));

    std::vector<AgentState> next_other_agent_states(num_other_agents);
    for (size_t i = 0; i < other_agent_states_.size(); ++i) {
      const auto &old_state = other_agent_states_[i];
      int new_x = old_state.x_pos + static_cast<int>(Actions::FORWARD);
      next_other_agent_states[i] = AgentState((new_x >= 0) ? new_x : 0, Actions::FORWARD);
    }

    // REWARD GENERATION
    EvaluationMap labels;
    std::vector<EvaluatorRuleLTL> next_automata(automata_);
    labels["other_goal_reached"] = (next_other_agent_states[0].x_pos >= ego_goal_reached_position);
    World next_world(ego_state_, other_agent_states_);
    for (auto le : label_evaluator_) {
      labels[le->get_label_str()] = le->evaluate(next_world);
    }
    const bool terminal = labels["ego_goal_reached"] || labels["collision"] || ego_out_of_map;
    rewards.resize(num_other_agents + 1);
    rewards[0] = Reward::Zero();
    for (auto aut : next_automata) {
      rewards[0](aut.get_type()) += aut.evaluate(labels);
      if (terminal) {
        rewards[0](aut.get_type()) += aut.final_reward();
      }
    }

    if (terminal) {
      rewards[0](static_cast<int>(RewardPriority::TIME)) = -time_penalty_;
    }

    //rewards[0](static_cast<int>(RewardPriority::TIME)) += -1.0f;
    // Extra penalty for driving backwards
    rewards[0](static_cast<int>(RewardPriority::TIME)) +=
        aconv(joint_action[ego_agent_idx]) == ActionType::BACKWARD ? -1.0f : 0.0f;

    return std::make_shared<CrossingState>(next_other_agent_states,
                                           next_ego_state,
                                           terminal,
                                           next_automata,
                                           label_evaluator_,
                                           time_penalty_ + 1.0f);
  }

  ActionIdx get_num_actions(AgentIdx agent_idx) const {
    if (agent_idx == ego_agent_idx) {
      return static_cast<size_t>(Actions::NUM); // WAIT, FORWARD, BACKWARD
    } else {
      return static_cast<size_t>(1);
    }
  }

  bool is_terminal() const {
    return terminal_;
  }

  const std::vector<AgentIdx> get_agent_idx() const {
    std::vector<AgentIdx> agent_idx(1);
    std::iota(agent_idx.begin(), agent_idx.end(), 0);
    return agent_idx; // adapt to number of agents
  }

  std::string sprintf() const {
    std::stringstream ss;
    ss << "Ego: x=" << ego_state_.x_pos;
    int i = 0;
    for (const auto &st : other_agent_states_) {
      ss << ", Ag" << i << ": x=" << st.x_pos;
      i++;
    }
    ss << std::endl;
    return ss.str();
  }

  bool ego_goal_reached() const {
    return ego_state_.x_pos >= ego_goal_reached_position;
  }

  int min_distance_to_ego() const {
    int min_dist = std::numeric_limits<int>::max();
    for (int i = 0; i < other_agent_states_.size(); ++i) {
      const auto dist = distance_to_ego(i);
      if (min_dist > dist) {
        min_dist = dist;
      }
    }
    return min_dist;
  }

  inline int distance_to_ego(const AgentIdx &other_agent_idx) const {
    return ego_state_.x_pos - other_agent_states_[other_agent_idx].x_pos;
  }

  typedef Actions ActionType;

  int get_ego_pos() const {
    return ego_state_.x_pos;
  }

 private:

  std::vector<AgentState> other_agent_states_;
  AgentState ego_state_;
  bool terminal_;
  std::vector<EvaluatorRuleLTL> automata_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator_;
  float time_penalty_;
};

const int CrossingState::crossing_point;
const int CrossingState::ego_goal_reached_position;
#endif
