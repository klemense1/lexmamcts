//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test/crossing_test/label_evaluator/evaluator_label_ego_range.h"

EvaluatorLabelEgoRange::EvaluatorLabelEgoRange(const std::string& label_str,
                                               int start, int end)
    : EvaluatorLabelBase(label_str), start_(start), end_(end) {}
bool EvaluatorLabelEgoRange::evaluate(const World& state) const {
  return (state.first.x_pos >= start_ && state.first.x_pos <= end_);
}
