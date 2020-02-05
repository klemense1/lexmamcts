// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef TEST_CROSSING_TEST_CROSSING_STATE_PARAMETER_H_
#define TEST_CROSSING_TEST_CROSSING_STATE_PARAMETER_H_

#include "ltl/rule_monitor.h"

using ltl::RewardPriority;

struct CrossingStateParameter {
  unsigned int num_other_agents;
  int state_x_length;
  int ego_goal_reached_position;
  int terminal_depth_;
  int crossing_point;

  size_t depth_prio;
  size_t speed_deviation_prio;
  size_t acceleration_prio;
  size_t potential_prio;

  float depth_weight;
  float speed_deviation_weight;
  float acceleration_weight;
  float potential_weight;

  size_t reward_vec_size;

  std::vector<int> action_map;

  bool merge;
};

CrossingStateParameter make_default_crossing_state_parameters();

#endif  // TEST_CROSSING_TEST_CROSSING_STATE_PARAMETER_H_
