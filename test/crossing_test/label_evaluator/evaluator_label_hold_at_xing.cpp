//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_hold_at_xing.h"

EvaluatorLabelHoldAtXing::EvaluatorLabelHoldAtXing(const std::string &label_str, const int crossing_point)
    : EvaluatorLabelBase(label_str), holding_point_(crossing_point - 1) {}
bool EvaluatorLabelHoldAtXing::evaluate(const World &state) const {
  return ((state.first.x_pos - static_cast<int>(aconv(state.first.last_action))) < holding_point_
      && state.first.x_pos >= holding_point_) || state.first.x_pos == holding_point_;
}
