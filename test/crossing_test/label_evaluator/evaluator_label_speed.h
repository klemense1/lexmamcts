//
// Created by luis on 26.10.19.
//

#ifndef MAMCTS_EVALUATOR_LABEL_SPEED_HPP
#define MAMCTS_EVALUATOR_LABEL_SPEED_HPP

#include <string>
#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelSpeed : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelSpeed(const std::string &label_str = "speeding");;

  bool evaluate(const World &state) const override;
};

#endif //MAMCTS_EVALUATOR_LABEL_SPEED_HPP
