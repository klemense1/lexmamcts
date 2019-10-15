//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_

#include "common.hpp"
#include "test/crossing_test/evaluator_label_base.hpp"

class EvaluatorLabelCollision : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelCollision(const std::string label_str, const int crossing_point)
      : EvaluatorLabelBase(label_str), crossing_point_(crossing_point) {};
  virtual bool evaluate(const World &state) const override;
 private:
  int crossing_point_;
};

bool EvaluatorLabelCollision::evaluate(const World &state) const {
  bool collision = false;
  for (const auto &agent: state.second) {
    if ((state.first.x_pos - static_cast<int>(aconv(state.first.last_action))) < crossing_point_ &&
        state.first.x_pos >= crossing_point_ &&
        agent.x_pos == crossing_point_) {
      collision = true;
    }
  }
  return collision;
}

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
