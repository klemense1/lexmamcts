//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_other_near.h"

EvaluatorLabelOtherNear::EvaluatorLabelOtherNear(const std::string &label_str)
    : EvaluatorLabelBase(label_str) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelOtherNear::evaluate(
    const World &state) const {
  for (auto agent : state.second) {
    if (abs(state.first.x_pos - agent.x_pos) <= 2) {
      return {{get_label(), true}};
    }
  }
  return {{get_label(), false}};
}
