//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_at_from.h"
#include "test/crossing_test/label_evaluator/evaluator_label_at_position.h"

bool EvaluatorLabelAtFrom::evaluate(const World& state) const {
  EvaluatorLabelAtPosition at_point("DUMMYF", point_);
  auto agents = state.second;
  agents.emplace_back(state.first);
  for(const auto &agent : agents) {
    if(agent.lane == origin_lane_ && at_point.evaluate(World(agent,std::vector<AgentState>()))) {
      return true;
    }
  }
  return false;
}
