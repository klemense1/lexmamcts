// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "mcts/mcts.h"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/viewer.h"

const int CHAIN_LENGTH = 21; /* 10 is crossing point (21-1)/2+1 */


void CrossingState::draw(mcts::Viewer *viewer) const {
    // draw map ( crossing point is always at zero)
    const float state_draw_dst = 1.0f;
    const float linewidth = 2;
    const float state_draw_size = 50;

    // draw lines equally spaced angles with small points
    // indicating states and larger points indicating the current state
    const float angle_delta = M_PI / (NUM_OTHER_AGENTS + 2); // one for ego
    const float line_radius = state_draw_dst * (CHAIN_LENGTH - 1) / 2.0f;
    for (int i = 0; i < NUM_OTHER_AGENTS + 1; ++i) {
        float start_angle = 1.5 * M_PI - (i + 1) * angle_delta;
        float end_angle = start_angle + M_PI;
        std::pair<float, float> line_x{cos(start_angle) * line_radius, cos(end_angle) * line_radius};
        std::pair<float, float> line_y{sin(start_angle) * line_radius, sin(end_angle) * line_radius};
        std::tuple<float, float, float, float> color{0, 0, 0, 0};
        const std::tuple<float, float, float, float> gray{0.5, 0.5, 0.5, 0};
        std::tuple<float, float, float, float> current_color = gray;
        // Differentiate between ego and other agents
        AgentState state;
        if (i == std::floor(NUM_OTHER_AGENTS / 2)) {
            state = agent_states_[ego_agent_idx];
            color = {0.8, 0, 0, 0};
        } else {
            AgentIdx agt_idx = i;
            if (i > std::floor(NUM_OTHER_AGENTS / 2)) {
                agt_idx = i - 1;
            }
            state = agent_states_[agt_idx + 1];
        }
        viewer->drawLine(line_x, line_y,
                         linewidth, gray);

        // Draw current states
        for (int y = 0; y < CHAIN_LENGTH; ++y) {
            const auto px = line_x.first + (line_x.second - line_x.first) * static_cast<float>(y) /
                static_cast<float>(CHAIN_LENGTH - 1);
            const auto py = line_y.first + (line_y.second - line_y.first) * static_cast<float>(y) /
                static_cast<float>(CHAIN_LENGTH - 1);
            float pointsize_temp = state_draw_size * 4;
            if (state.x_pos == y) {
                //pointsize_temp *= factor_draw_current_state;
                current_color = color;
            } else {
                current_color = gray;
            }
            viewer->drawPoint(px, py, pointsize_temp, current_color);
        }

    }

}
Reward CrossingState::get_action_cost(ActionIdx action) {
    Reward reward = Reward::Zero();
    reward(static_cast<int>(RewardPriority::TIME)) += -1.0f;
    switch (aconv(action)) {
        case Actions::FORWARD :reward(static_cast<int>(RewardPriority::EFFICIENCY)) = -1.0f;
            break;
        case Actions::WAIT:reward(static_cast<int>(RewardPriority::EFFICIENCY)) = 0.0f;
            break;
        case Actions::FASTFORWARD:reward(static_cast<int>(RewardPriority::EFFICIENCY)) = -2.0f;
            break;
        default:reward(static_cast<int>(RewardPriority::EFFICIENCY)) = 0.0f;
            break;
    }
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

const int CrossingState::crossing_point;
const int CrossingState::ego_goal_reached_position;
