//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_collision.h"

EvaluatorLabelCollision::EvaluatorLabelCollision(const std::string &label_str, const int crossing_point)
    : EvaluatorLabelBase(label_str), crossing_point_(crossing_point) {}
bool EvaluatorLabelCollision::evaluate(const World &state) const {
  for (const auto &agent: state.second) {
    if (((state.first.x_pos - static_cast<int>(aconv(state.first.last_action))) < crossing_point_
        && state.first.x_pos >= crossing_point_
        && (agent.x_pos - static_cast<int>(aconv(agent.last_action))) < crossing_point_
        && agent.x_pos >= crossing_point_)
        || (agent.x_pos == crossing_point_ && state.first.x_pos == crossing_point_)) {
      return true;
    }
  }
  return false;
}
