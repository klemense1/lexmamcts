//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_in_direct_front.h"
EvaluatorLabelInDirectFront::EvaluatorLabelInDirectFront(
    const std::string& label_str)
    : EvaluatorLabelMultiAgent(label_str) {}
bool EvaluatorLabelInDirectFront::evaluate_agent(const World& state,
                                                 int agent_id) const {
  if (state.second[agent_id].lane != state.first.lane ||
      state.second[agent_id].x_pos <= state.first.x_pos) {
    // Not on same lane or behind
    return false;
  }
  for (const auto& agent : state.second) {
    if(agent == state.second[agent_id]) {
      continue;
    }
    if (agent.x_pos >= state.first.x_pos &&
        agent.x_pos <= state.second[agent_id].x_pos &&
        agent.lane == state.first.lane) {
      // Found another agent that is closer or equal
      return false;
    }
  }
  return true;
}
