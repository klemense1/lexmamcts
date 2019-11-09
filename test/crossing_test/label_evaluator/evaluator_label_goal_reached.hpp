//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_GOAL_REACHED_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_GOAL_REACHED_HPP_

#include "test/crossing_test/common.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_base.hpp"

class EvaluatorLabelGoalReached : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelGoalReached(const std::string label_str, const int goal_position)
      : EvaluatorLabelBase(label_str), goal_position_(goal_position) {};
  virtual bool evaluate(const World &state) const override;
 private:
  int goal_position_;
};

bool EvaluatorLabelGoalReached::evaluate(const World &state) const {
  return (state.first.x_pos >= goal_position_);
}

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_GOAL_REACHED_HPP_
