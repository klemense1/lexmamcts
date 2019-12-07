//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_range.h"
EvaluatorLabelRange::EvaluatorLabelRange(const std::string& label_str,
                                         int start, int end)
    : EvaluatorLabelBase(label_str), start_(start), end_(end) {}
bool EvaluatorLabelRange::evaluate(const World& state) const {
  auto agents = state.second;
  for(const auto &agent : agents) {
    if(agent.x_pos >= start_ && agent.x_pos <= end_) {
      return true;
    }
  }
  return false;
}
