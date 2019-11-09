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

CrossingState::CrossingState(Automata &automata,
                             const std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
                             const CrossingStateParameter &parameters)
    : agent_states_(parameters.num_other_agents + 1),
      terminal_(false),
      automata_(automata),
      label_evaluator_(label_evaluator),
      depth_(0),
      parameters_(parameters){
  for (auto &state : agent_states_) {
    state = AgentState();
  }
}

CrossingState::CrossingState(const std::vector<AgentState> &agent_states,
                             const bool terminal,
                             Automata &automata,
                             const std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> &label_evaluator,
                             const CrossingStateParameter &parameters,
                             int depth)
    : agent_states_(agent_states),
      terminal_(terminal),
      automata_(automata),
      label_evaluator_(label_evaluator),
      depth_(depth),
      parameters_(parameters) {}

std::shared_ptr<CrossingState> CrossingState::execute(const JointAction &joint_action, std::vector<Reward> &rewards) const {

  EvaluationMap labels;
  Automata next_automata(automata_);
  World next_world;
  bool terminal;
  std::vector<AgentState> next_agent_states(agent_states_.size());
  rewards.resize(parameters_.num_other_agents + 1);

  // CALCULATE NEXT STATE
  for (size_t i = 0; i < agent_states_.size(); ++i) {
    const auto &old_state = agent_states_[i];
    int new_x = old_state.x_pos + static_cast<int>(aconv(joint_action[i]));
    next_agent_states[i] = AgentState(new_x, aconv(joint_action[i]));
  }
  labels["ego_out_of_map"] = false;
  if (next_agent_states[ego_agent_idx].x_pos < 0) {
    labels["ego_out_of_map"] = true;
  }

  // REWARD GENERATION
  // For each agent
  for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
    // Labeling
    std::vector<AgentState> next_other_agents(next_agent_states);
    // TODO: Improve efficiency
    next_other_agents.erase(next_other_agents.begin() + agent_idx);
    // Create perspective from current agent
    next_world = World(next_agent_states[agent_idx], next_other_agents);

    for (auto le : label_evaluator_) {
      labels[le->get_label_str()] = le->evaluate(next_world);
    }
    assert(ego_agent_idx == 0);
    if (agent_idx == ego_agent_idx) {
      terminal = labels["goal_reached"] || labels["collision"] || (depth_ + 1 >= parameters_.terminal_depth_);
    }
    rewards[agent_idx] = Reward::Zero();

    // Automata transit
    for (EvaluatorRuleLTL &aut : (next_automata[agent_idx])) {
      rewards[agent_idx](aut.get_type()) += aut.evaluate(labels);
    }
    rewards[agent_idx] += get_action_cost(joint_action[agent_idx]);
    labels.clear();
  } // End for each agent

  return std::make_shared<CrossingState>(next_agent_states, terminal, next_automata, label_evaluator_, parameters_, depth_ + 1);
}
Reward CrossingState::get_action_cost(ActionIdx action) const {
  Reward reward = Reward::Zero();
  reward(parameters_.depth_prio) += -1.0f * parameters_.depth_weight;
  //  switch (aconv(action)) {
  //      case Actions::FORWARD :reward(static_cast<int>(RewardPriority::EFFICIENCY)) = -1.0f;
  //          break;
  //      case Actions::WAIT:reward(static_cast<int>(RewardPriority::EFFICIENCY)) = 0.0f;
  //          break;
  //      case Actions::FASTFORWARD:reward(static_cast<int>(RewardPriority::EFFICIENCY)) = -2.0f;
  //          break;
  //    case Actions::BACKWARD:reward(static_cast<int>(RewardPriority::EFFICIENCY)) = -3.0f;
  //      break;
  //      default:reward(static_cast<int>(RewardPriority::EFFICIENCY)) = 0.0f;
  //          break;
  //  }
  reward(parameters_.speed_deviation_prio) =
      -std::abs(static_cast<int>(aconv(action)) - static_cast<int>(Actions::FORWARD)) * parameters_.speed_deviation_weight;
  return reward;
}
void CrossingState::update_rule_belief() {
  for (size_t agent_idx = 0; agent_idx < automata_.size(); agent_idx++) {
    if (agent_idx == ego_agent_idx) {
      continue;
    }
    for (auto &aut : automata_[agent_idx]) {
      aut.update_belief();
      LOG(INFO) << aut;
    }
  }
}
void CrossingState::reset_violations() {
  for (auto &agent : automata_) {
    for (auto &aut : agent) {
      aut.reset_violation();
    }
  }
}
std::vector<Reward> CrossingState::get_final_reward() const {
  std::vector<Reward> rewards(agent_states_.size(), Reward::Zero());
  for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
    // Automata transit
    for (EvaluatorRuleLTL const &aut : (automata_[agent_idx])) {
      rewards[agent_idx](aut.get_type()) += aut.get_final_reward();
    }
    // Reward for goal proximity
    //rewards[agent_idx](RewardPriority::GOAL) += ALPHA * fmin(agent_states_[agent_idx].x_pos, ego_goal_reached_position);
  }
  return rewards;
}
bool CrossingState::is_terminal() const {
  return terminal_;
}
ActionIdx CrossingState::get_num_actions(AgentIdx agent_idx) const {
  return static_cast<size_t>(Actions::NUM);
}
const std::vector<AgentIdx> CrossingState::get_agent_idx() const {
  std::vector<AgentIdx> agent_idx(parameters_.num_other_agents + 1);
  std::iota(agent_idx.begin(), agent_idx.end(), 0);
  return agent_idx; // adapt to number of agents
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
  return agent_states_[agent_idx].last_action;
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
  for (int i = 0; i < parameters_.num_other_agents + 1; ++i) {
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
