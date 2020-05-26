//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_multi_agent.h"
EvaluatorLabelMultiAgent::EvaluatorLabelMultiAgent(const std::string& label_str)
    : EvaluatorLabelBase(label_str) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelMultiAgent::evaluate(
    const World& state) const {
  std::vector<std::pair<ltl::Label, bool>> agent_labels;
  for (size_t i = 0; i < state.second.size(); ++i) {
    agent_labels.emplace_back(std::make_pair(
        get_agent_label(state.second[i].id), evaluate_agent(state, i)));
  }
  return agent_labels;
}
