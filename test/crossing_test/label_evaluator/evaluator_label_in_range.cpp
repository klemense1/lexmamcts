//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_in_range.h"

EvaluatorLabelInRange::EvaluatorLabelInRange(const std::string& label_str,
                                             int range_start, int range_end)
    : EvaluatorLabelMultiAgent(label_str),
      range_start_(range_start),
      range_end_(range_end) {}
bool EvaluatorLabelInRange::evaluate_agent(const World& state,
                                           int agent_id) const {
  return state.second[agent_id].x_pos >= range_start_ &&
         state.second[agent_id].x_pos <= range_end_;
}
