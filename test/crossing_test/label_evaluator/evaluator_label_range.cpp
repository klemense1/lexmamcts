//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_range.h"
bool EvaluatorLabelRange::evaluate(const World& state) const {
  auto agents = state.second;
  agents.emplace_back(state.first);
  for(const auto &agent : agents) {
    if(agent.lane == lane_ && agent.x_pos >= start_ && agent.x_pos <= end_) {
      return true;
    }
  }
  return false;
}
