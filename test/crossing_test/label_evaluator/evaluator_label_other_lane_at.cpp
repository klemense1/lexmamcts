//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "test/crossing_test/label_evaluator/evaluator_label_other_lane_at.h"
#include "test/crossing_test/label_evaluator/evaluator_label_at_position.h"

bool EvaluatorLabelOtherLaneAt::evaluate(const World& state) const {
  EvaluatorLabelAtPosition at_point("DUMMYF", point_);
  for(const auto &agent : state.second) {
    if(agent.lane != state.first.lane && at_point.evaluate(World(agent,std::vector<AgentState>()))) {
      return true;
    }
  }
  return false;
}
EvaluatorLabelOtherLaneAt::EvaluatorLabelOtherLaneAt(const std::string& label_str,
                                           int point)
    : EvaluatorLabelBase(label_str), point_(point) {}
