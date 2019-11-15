//
// Created by luis on 15.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_HOLD_AT_XING_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_HOLD_AT_XING_HPP_

#include <string>
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelHoldAtXing : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelHoldAtXing(const std::string &label_str, const int crossing_point);;
  bool evaluate(const World &state) const override;
 private:
  int holding_point_;
};

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_HOLD_AT_XING_HPP_
