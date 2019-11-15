//
// Created by luis on 15.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_

#include <string>
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelOtherNear : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelOtherNear(const std::string &label_str);;
  bool evaluate(const World &state) const override;
};

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_
