//
// Created by Luis Gressenbuch on 06.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_AT_FROM_H_
#define MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_AT_FROM_H_

#include "ltl_evaluator/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelAtFrom : public EvaluatorLabelBase<World>{
 public:
  EvaluatorLabelAtFrom(const std::string& label_str, int point);
  bool evaluate(const World& state) const override;
 private:
  int point_;
};

#endif  //  MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_AT_FROM_H_
