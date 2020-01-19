//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_other_goal_reached.h"

EvaluatorLabelOtherGoalReached::EvaluatorLabelOtherGoalReached(
    const std::string &label_str, const int goal_position)
    : EvaluatorLabelBase(label_str), goal_position_(goal_position) {}
std::vector<std::pair<ltl::Label, bool>>
EvaluatorLabelOtherGoalReached::evaluate(const World &state) const {
  for (auto const &agent : state.second) {
    if (agent.x_pos >= goal_position_) {
      return {{get_label(), true}};
    }
  }
  return {{get_label(), false}};
}
