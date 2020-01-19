// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "test/crossing_test/crossing_state_parameter.h"
#include "test/crossing_test/common.hpp"

CrossingStateParameter make_default_crossing_state_parameters() {
  CrossingStateParameter p;
  p.num_other_agents = 1;
  p.state_x_length = 13;
  p.ego_goal_reached_position = 13;
  p.terminal_depth_ = 30;
  p.ALPHA = 10.0f;
  p.crossing_point = (p.state_x_length - 1) / 2 + 1;

  p.depth_prio = static_cast<size_t>(RewardPriority::GOAL);
  p.speed_deviation_prio = static_cast<size_t>(RewardPriority::GOAL);
  p.acceleration_prio = static_cast<size_t>(RewardPriority::GOAL);
  p.potential_prio = static_cast<size_t>(RewardPriority::GOAL);

  p.depth_weight = 1.0f;
  p.speed_deviation_weight = 1.0f;
  p.acceleration_weight = 1.0f;
  p.potential_weight = 20.0f;

  p.reward_vec_size = 5;

  p.action_map = {static_cast<int>(Actions::FORWARD),
                  static_cast<int>(Actions::WAIT),
                  static_cast<int>(Actions::FASTFORWARD),
                  static_cast<int>(Actions::BACKWARD)};

  p.merge = false;

  return p;
}
