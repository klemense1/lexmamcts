//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_RANGE_H_
#define MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_RANGE_H_

#include <string>
#include "test/crossing_test/common.hpp"
#include "ltl_evaluator/evaluator_label_base.h"

class EvaluatorLabelRange : public EvaluatorLabelBase<World>{
 public:
  EvaluatorLabelRange(const std::string& label_str, int lane, int start, int end)
      : EvaluatorLabelBase(label_str), lane_(lane), start_(start), end_(end) {}
  bool evaluate(const World& state) const override;
 private:
  int lane_;
  int start_;
  int end_;
};

#endif  //  MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_RANGE_H_
