//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_EGO_RANGE_H_
#define TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_EGO_RANGE_H_

#include <string>

#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelEgoRange : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelEgoRange(const std::string& label_str, int start, int end);
  bool evaluate(const World& state) const override;

 private:
  int start_;
  int end_;
};

#endif  // TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_EGO_RANGE_H_
