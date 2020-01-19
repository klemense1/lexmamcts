//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_goal_reached.h"

EvaluatorLabelGoalReached::EvaluatorLabelGoalReached(
    const std::string &label_str, const int goal_position)
    : EvaluatorLabelBase(label_str), goal_position_(goal_position) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelGoalReached::evaluate(
    const World &state) const {
  return {{get_label(), (state.first.x_pos >= goal_position_)}};
}
