//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_on_ego_lane.h"
EvaluatorLabelOnEgoLane::EvaluatorLabelOnEgoLane(const std::string& label_str)
    : EvaluatorLabelMultiAgent(label_str) {}
bool EvaluatorLabelOnEgoLane::evaluate_agent(const World& state,
                                             int agent_id) const {
  return state.first.lane == state.second[agent_id].lane;
}
