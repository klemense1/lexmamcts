//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test/crossing_test/label_evaluator/evaluator_label_other_at_ego_lane_at_pos.h"

#include <vector>

#include "test/crossing_test/label_evaluator/evaluator_label_at_position.h"

EvaluatorLabelOtherAtEgoLaneAtPos::EvaluatorLabelOtherAtEgoLaneAtPos(
    const std::string& label_str, int point)
    : EvaluatorLabelBase(label_str), point_(point) {}
bool EvaluatorLabelOtherAtEgoLaneAtPos::evaluate(const World& state) const {
  EvaluatorLabelAtPosition at_point("DUMMYF", point_);
  auto agents = state.second;
  for (const auto& agent : agents) {
    if (agent.lane == state.first.lane &&
        at_point.evaluate(World(agent, std::vector<AgentState>()))) {
      return true;
    }
  }
  return false;
}
