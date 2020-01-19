//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test/crossing_test/label_evaluator/evaluator_label_other_lane_at.h"
#include "test/crossing_test/label_evaluator/evaluator_label_at_position.h"

EvaluatorLabelOtherLaneAt::EvaluatorLabelOtherLaneAt(
    const std::string& label_str, int point)
    : EvaluatorLabelBase(label_str), point_(point) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelOtherLaneAt::evaluate(
    const World& state) const {
  EvaluatorLabelAtPosition at_point("DUMMYF", point_);
  for (const auto& agent : state.second) {
    if ((agent.lane != state.first.lane) &&
        at_point.evaluate(World(agent, std::vector<AgentState>()))[0].second) {
      return {{get_label(), true}};
    }
  }
  return {{get_label(), false}};
}
