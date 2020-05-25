// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "test/crossing_test/crossing_state_parameter.h"
#include "test/crossing_test/common.hpp"

CrossingStateParameter MakeDefaultCrossingStateParameters() {
  CrossingStateParameter p;
  p.num_other_agents = 1;
  p.state_x_length = 100;
  p.ego_goal_reached_position = 100;
  p.terminal_depth_ = 12;
  p.crossing_point = 8;

  p.depth_prio = 2;
  p.speed_deviation_prio = 2;
  p.acceleration_prio = 2;
  p.potential_prio = 2;

  p.depth_weight = 0.0f;
  p.speed_deviation_weight = 2.0f;
  p.acceleration_weight = 2.0f;
  p.potential_weight = 0.0f;

  p.reward_vec_size = 3;

  p.action_map = {static_cast<int>(Actions::FORWARD),
                  static_cast<int>(Actions::WAIT),
                  static_cast<int>(Actions::FASTFORWARD),
                  static_cast<int>(Actions::BACKWARD)};

  p.merge = false;

  return p;
}
