//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_other_range.h"
EvaluatorLabelOtherRange::EvaluatorLabelOtherRange(const std::string& label_str,
                                                   int start, int end)
    : EvaluatorLabelBase(label_str), start_(start), end_(end) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelOtherRange::evaluate(
    const World& state) const {
  for (const auto& agent : state.second) {
    if (agent.x_pos >= start_ && agent.x_pos <= end_) {
      return {{get_label(), true}};
    }
  }
  return {{get_label(), false}};
}
