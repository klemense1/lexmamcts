//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_CROSSING_STATE_PARAMETER_H_
#define MAMCTS_TEST_CROSSING_TEST_CROSSING_STATE_PARAMETER_H_

#include "ltl_evaluator/evaluator_rule_ltl.h"

using modules::models::behavior::RewardPriority;

struct CrossingStateParameter {
  unsigned int num_other_agents;
  int state_x_length;
  int ego_goal_reached_position;
  int terminal_depth_;
  float ALPHA;
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
};

CrossingStateParameter make_default_crossing_state_parameters();

#endif //MAMCTS_TEST_CROSSING_TEST_CROSSING_STATE_PARAMETER_H_
