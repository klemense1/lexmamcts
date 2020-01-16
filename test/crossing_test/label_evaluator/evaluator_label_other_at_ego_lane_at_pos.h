//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_OTHER_AT_EGO_LANE_AT_POS_H_
#define TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_OTHER_AT_EGO_LANE_AT_POS_H_

#include <string>

#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelOtherAtEgoLaneAtPos : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelOtherAtEgoLaneAtPos(const std::string& label_str, int point);
  bool evaluate(const World& state) const override;

 private:
  int point_;
};

#endif  // TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_OTHER_AT_EGO_LANE_AT_POS_H_
