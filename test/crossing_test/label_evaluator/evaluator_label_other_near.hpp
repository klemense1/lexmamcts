//
// Created by luis on 15.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_

#include <string>
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_base.hpp"

class EvaluatorLabelOtherNear : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelOtherNear(const std::string label_str)
      : EvaluatorLabelBase(label_str) {};
  virtual bool evaluate(const World &state) const override;
};

bool EvaluatorLabelOtherNear::evaluate(const World &state) const {
  for (auto agent : state.second) {
    if (abs(state.first.x_pos - agent.x_pos) < 2) {
      return true;
    }
  }
  return false;
}

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_
