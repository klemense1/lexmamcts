//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "crossing_state_parameter.h"

CrossingStateParameter make_default_crossing_state_parameters() {
  CrossingStateParameter p;
  p.num_other_agents = 1;
  p.state_x_length = 13;
  p.ego_goal_reached_position = 13;
  p.terminal_depth_ = 30;
  p.ALPHA = 10.0f;
  p.crossing_point = (p.state_x_length - 1) / 2 + 1;

  p.depth_prio = static_cast<size_t >(RewardPriority::TIME);
  p.speed_deviation_prio = static_cast<size_t >(RewardPriority::GOAL);

  p.depth_weight = 1.0f;
  p.speed_deviation_weight = 1.0f;

  return p;
};