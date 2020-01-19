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
std::vector<std::pair<ltl::Label, bool>>
EvaluatorLabelOtherAtEgoLaneAtPos::evaluate(const World& state) const {
  EvaluatorLabelAtPosition at_point("DUMMYF", point_);
  auto agents = state.second;
  for (const auto& agent : agents) {
    if (agent.lane == state.first.lane && state.first.x_pos <= point_ &&
        at_point.evaluate(World(agent, std::vector<AgentState>()))[0].second) {
      return {{get_label(), true}};
    }
  }
  return {{get_label(), false}};
}
