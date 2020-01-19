//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_speed.h"

EvaluatorLabelSpeed::EvaluatorLabelSpeed(const std::string &label_str)
    : EvaluatorLabelBase(label_str) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelSpeed::evaluate(
    const World &state) const {
  return {{get_label(), (state.first.last_action ==
                         static_cast<int>(Actions::FASTFORWARD))}};
}
