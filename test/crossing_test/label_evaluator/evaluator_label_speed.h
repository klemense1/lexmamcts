//
// Created by luis on 26.10.19.
//

#ifndef MAMCTS_EVALUATOR_LABEL_SPEED_HPP
#define MAMCTS_EVALUATOR_LABEL_SPEED_HPP

#include <string>
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelSpeed : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelSpeed(const std::string &label_str = "speeding");;

  bool evaluate(const World &state) const override;
};

#endif //MAMCTS_EVALUATOR_LABEL_SPEED_HPP
