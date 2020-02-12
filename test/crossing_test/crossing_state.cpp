// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include <tuple>
#include <utility>

#include "mcts/mcts.h"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/viewer.h"

CrossingState::CrossingState(RuleStateMap rule_state_map,
                             std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
                             const CrossingStateParameter &parameters)
    : agent_states_(parameters.num_other_agents + 1),
      terminal_(false), rule_state_map_(std::move(rule_state_map)), label_evaluator_(std::move(label_evaluator)),
      depth_(0),
      parameters_(parameters),
      terminal_agents_(parameters.num_other_agents + 1, false) {
  for (auto &state : agent_states_) {
    state = AgentState();
  }
  //  agent_states_[1].x_pos = 1;
}

CrossingState::CrossingState(std::vector<AgentState> agent_states,
                             const bool terminal,
                             RuleStateMap rule_state_map,
                             std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
    const CrossingStateParameter &parameters, int depth,
    const std::vector<bool> &terminal_agents)
    : agent_states_(std::move(agent_states)),
      terminal_(terminal),
      rule_state_map_(std::move(rule_state_map)),
                                          label_evaluator_(std::move(label_evaluator)),
                                          depth_(depth),
      parameters_(parameters),
      terminal_agents_(terminal_agents) {}

std::shared_ptr<CrossingState> CrossingState::execute(const JointAction &joint_action, std::vector<Reward> &rewards) const {
  EvaluationMap labels;
  RuleStateMap next_automata(rule_state_map_);
  World next_world;
  rewards.resize(parameters_.num_other_agents + 1);

  // CALCULATE SUCCESSOR AGENT STATES
  std::vector<AgentState> next_agent_states = step(joint_action);

  // REWARD GENERATION
  // For each agent
  std::vector<bool> agent_terminal(agent_states_.size(), false);
  for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
    if (terminal_agents_[agent_idx]) {
      agent_terminal[agent_idx] = true;
      continue;
    }
    // Labeling
    std::vector<AgentState> next_other_agents(next_agent_states);
    next_other_agents.erase(next_other_agents.begin() + agent_idx);
    // Create perspective from current agent
    next_world = World(next_agent_states[agent_idx], next_other_agents);

    for (const auto &le : label_evaluator_) {
      auto new_labels = le->evaluate(next_world);
      labels.insert(new_labels.begin(), new_labels.end());
    }
    rewards[agent_idx] = Reward::Zero(parameters_.reward_vec_size);

    // Automata transit
    for (auto &aut : (next_automata[agent_idx])) {
      rewards[agent_idx](aut.second.get_priority()) += aut.second.get_automaton()->evaluate(labels, aut.second);
    }

    rewards[agent_idx] += get_action_cost(joint_action[agent_idx], agent_idx);
    auto obstacle_rule_it = next_automata[agent_idx].find(Rule::OBSTACLE);
    bool collision_obstacle =
        obstacle_rule_it != next_automata[agent_idx].end() &&
        obstacle_rule_it->second.get_violation_count() > 0;
    agent_terminal[agent_idx] =
        agent_terminal[agent_idx] || labels[Label("collision")] ||
        collision_obstacle || (depth_ + 1 >= parameters_.terminal_depth_);
    labels.clear();
  }  // End for each agent
  return std::make_shared<CrossingState>(
      next_agent_states, agent_terminal[0], next_automata, label_evaluator_,
      parameters_, depth_ + 1, agent_terminal);
}
std::vector<AgentState> CrossingState::step(
    const JointAction &joint_action) const {
  std::vector<AgentState> next_agent_states(agent_states_.size());
  for (size_t i = 0; i < this->agent_states_.size(); ++i) {
    if (this->terminal_agents_[i]) {
      next_agent_states[i] = this->agent_states_[i];
    } else {
      const auto &old_state = this->agent_states_[i];
      int new_x =
          old_state.x_pos + this->parameters_.action_map[joint_action[i]];
      int new_lane = old_state.lane;
      if (parameters_.merge && new_x >= parameters_.crossing_point) {
        // If merging is activated, merge all agents to the same lane in and
        // after crossing point.
        new_lane = 0;
      } else if (parameters_.merge && new_x < parameters_.crossing_point) {
        // When driving backwards, restore the initial lane
        new_lane = old_state.init_lane;
      }
      next_agent_states[i] = old_state;
      next_agent_states[i].x_pos = new_x;
      next_agent_states[i].last_action =
          this->parameters_.action_map[joint_action[i]];
      next_agent_states[i].lane = new_lane;
    }
  }
  return next_agent_states;
}
Reward CrossingState::get_action_cost(ActionIdx action, AgentIdx agent_idx) const {
  Reward reward = Reward::Zero(parameters_.reward_vec_size);
  reward(parameters_.depth_prio) += -1.0f * parameters_.depth_weight;
  reward(parameters_.speed_deviation_prio) +=
      -std::abs(parameters_.action_map[action] -
                static_cast<int>(Actions::FORWARD)) *
      parameters_.speed_deviation_weight;
  reward(parameters_.acceleration_prio) +=
      -std::pow(
          parameters_.action_map[action] - agent_states_[agent_idx].last_action,
          2) * parameters_.acceleration_weight;
  return reward;
}
Reward CrossingState::get_shaping_reward(const AgentState &agent_state) const {
  Reward reward = Reward::Zero(parameters_.reward_vec_size);
  // Potential for goal distance
  reward(parameters_.potential_prio) += -parameters_.potential_weight * std::abs(parameters_.ego_goal_reached_position - agent_state.x_pos);
  return reward;
}
void CrossingState::update_rule_belief() {
  for (size_t agent_idx = 0; agent_idx < rule_state_map_.size(); agent_idx++) {
    if (agent_idx == ego_agent_idx) {
      continue;
    }
    for (auto &aut : rule_state_map_[agent_idx]) {
      aut.second.get_automaton()->update_belief(aut.second);
      LOG(INFO) << aut.second;
    }
  }
}
void CrossingState::reset_violations() {
  for (auto &agent : rule_state_map_) {
    for (auto &aut : agent) {
      aut.second.reset_violations();
    }
  }
}
std::vector<Reward> CrossingState::get_final_reward() const {
  std::vector<Reward> rewards(agent_states_.size(), Reward::Zero(parameters_.reward_vec_size));
  for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
    // Automata transit
    for (const auto &aut : (rule_state_map_[agent_idx])) {
      rewards[agent_idx](aut.second.get_priority()) += aut.second.get_automaton()->get_final_reward(aut.second);
    }
  }
  return rewards;
}
bool CrossingState::is_terminal() const {
  return terminal_;
}
ActionIdx CrossingState::get_num_actions(AgentIdx agent_idx) const {
  if (terminal_agents_[agent_idx]) {
    return static_cast<ActionIdx>(1);
  }
  return static_cast<ActionIdx>(parameters_.action_map.size());
}
const std::vector<AgentIdx> CrossingState::get_agent_idx() const {
  std::vector<AgentIdx> agent_idx(parameters_.num_other_agents + 1);
  std::iota(agent_idx.begin(), agent_idx.end(), 0);
  return agent_idx;  // adapt to number of agents
}
std::string CrossingState::sprintf() const {
  std::stringstream ss;
  ss << "Ego: x=" << agent_states_[ego_agent_idx].x_pos;
  for (size_t i = 1; i < agent_states_.size(); ++i) {
    ss << ", Ag" << i << ": x=" << agent_states_[i].x_pos;
  }
  ss << std::endl;
  return ss.str();
}
bool CrossingState::ego_goal_reached() const {
  return agent_states_[ego_agent_idx].x_pos >= parameters_.ego_goal_reached_position;
}
void CrossingState::reset_depth() {
  depth_ = 0;
}
const std::vector<AgentState> &CrossingState::get_agent_states() const {
  return agent_states_;
}
int CrossingState::get_ego_pos() const {
  return agent_states_[ego_agent_idx].x_pos;
}
template<typename ActionType>
ActionType CrossingState::get_last_action(const AgentIdx &agent_idx) const {
  return static_cast<ActionType>(agent_states_[agent_idx].last_action);
}
std::shared_ptr<CrossingState> CrossingState::clone() const {
  return std::make_shared<CrossingState>(*this);
}
void CrossingState::draw(mcts::Viewer *viewer) const {
  // draw map ( crossing point is always at zero)
  const float state_draw_dst = 1.0f;
  const float linewidth = 2;
  const float state_draw_size = 50;

  // draw lines equally spaced angles with small points
  // indicating states and larger points indicating the current state
  const float angle_delta = M_PI / (parameters_.num_other_agents + 2);  // one for ego
  const float line_radius = state_draw_dst * (parameters_.state_x_length - 1) / 2.0f;
  for (size_t i = 0; i < parameters_.num_other_agents + 1; ++i) {
    float start_angle = 1.5 * M_PI - (i + 1) * angle_delta;
    float end_angle = start_angle + M_PI;
    std::pair<float, float> line_x{cos(start_angle) * line_radius, cos(end_angle) * line_radius};
    std::pair<float, float> line_y{sin(start_angle) * line_radius, sin(end_angle) * line_radius};
    std::tuple<float, float, float, float> color{0, 0, 0, 0};
    const std::tuple<float, float, float, float> gray{0.5, 0.5, 0.5, 0};
    std::tuple<float, float, float, float> current_color = gray;
    // Differentiate between ego and other agents
    AgentState state;
    if (i == std::floor(parameters_.num_other_agents / 2)) {
      state = agent_states_[ego_agent_idx];
      color = {0.8, 0, 0, 0};
    } else {
      AgentIdx agt_idx = i;
      if (i > std::floor(parameters_.num_other_agents / 2)) {
        agt_idx = i - 1;
      }
      state = agent_states_[agt_idx + 1];
    }
    viewer->drawLine(line_x, line_y, linewidth, gray);

    // Draw current states
    for (int y = 0; y < parameters_.state_x_length; ++y) {
      const auto px = line_x.first
          + (line_x.second - line_x.first) * static_cast<float>(y) / static_cast<float>(parameters_.state_x_length - 1);
      const auto py = line_y.first
          + (line_y.second - line_y.first) * static_cast<float>(y) / static_cast<float>(parameters_.state_x_length - 1);
      float pointsize_temp = state_draw_size * 4;
      if (state.x_pos == y) {
        current_color = color;
      } else {
        current_color = gray;
      }
      viewer->drawPoint(px, py, pointsize_temp, current_color);
    }
  }
}
const CrossingStateParameter &CrossingState::get_parameters() const {
  return parameters_;
}
const RuleStateMap &CrossingState::get_rule_state_map() const {
  return rule_state_map_;
}
EvaluationMap CrossingState::get_agent_labels(AgentIdx agent_idx) const {
  EvaluationMap labels;
  std::vector<AgentState> next_other_agents(agent_states_);
  next_other_agents.erase(next_other_agents.begin() + agent_idx);
  // Create perspective from current agent
  World next_world(agent_states_[agent_idx], next_other_agents);

  for (const auto &le : label_evaluator_) {
    auto new_labels = le->evaluate(next_world);
    labels.insert(new_labels.begin(), new_labels.end());
  }
  return labels;
}
