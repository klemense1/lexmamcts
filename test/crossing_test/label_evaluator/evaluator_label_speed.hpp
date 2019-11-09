//
// Created by luis on 26.10.19.
//

#ifndef MAMCTS_EVALUATOR_LABEL_SPEED_HPP
#define MAMCTS_EVALUATOR_LABEL_SPEED_HPP

#include <string>
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_base.hpp"

class EvaluatorLabelSpeed : public EvaluatorLabelBase<World> {
public:
  EvaluatorLabelSpeed(const std::string label_str = "speeding") : EvaluatorLabelBase(label_str) {};

  virtual bool evaluate(const World &state) const override;
};

bool EvaluatorLabelSpeed::evaluate(const World &state) const {
  return (state.first.last_action == Actions::FASTFORWARD);
}


#endif //MAMCTS_EVALUATOR_LABEL_SPEED_HPP
