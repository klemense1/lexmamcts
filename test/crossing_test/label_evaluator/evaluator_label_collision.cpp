//
// Created by Luis Gressenbuch on 15.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "evaluator_label_collision.h"
#include "test/crossing_test/label_evaluator/evaluator_label_at_position.h"

EvaluatorLabelCollision::EvaluatorLabelCollision(const std::string &label_str,
                                                 const int crossing_point)
    : EvaluatorLabelBase(label_str), crossing_point_(crossing_point) {}
std::vector<std::pair<ltl::Label, bool>> EvaluatorLabelCollision::evaluate(
    const World &state) const {
  EvaluatorLabelAtPosition at_xing("DUMMYF", crossing_point_);
  EvaluatorLabelAtPosition at_ego("DUMMYG", state.first.x_pos);
  bool ego_at_xing = at_xing.evaluate(state)[0].second;
  World other_world(state.first, std::vector<AgentState>());
  for (const auto &agent : state.second) {
    other_world.first = agent;
    bool agent_at_xing = at_xing.evaluate(other_world)[0].second;
    if ((ego_at_xing && agent_at_xing) ||
        ((agent.lane == state.first.lane) &&
         at_ego.evaluate(other_world)[0].second)) {
      return {{get_label(), true}};
    }
  }
  return {{get_label(), false}};
}
