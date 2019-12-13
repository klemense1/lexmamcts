//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_at_position.h"

EvaluatorLabelAtPosition::EvaluatorLabelAtPosition(const std::string &label_str, const int position)
    : EvaluatorLabelBase(label_str), position_(position) {}
bool EvaluatorLabelAtPosition::evaluate(const World &state) const {
  return ((state.first.x_pos - static_cast<int>(state.first.last_action)) < position_
      && state.first.x_pos >= position_) || state.first.x_pos == position_;
}
