//
// Created by Luis Gressenbuch on 14.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_OTHER_GOAL_REACHED_HPP_
#define MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_OTHER_GOAL_REACHED_HPP_

#include "test/crossing_test/common.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelOtherGoalReached : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelOtherGoalReached(const std::string &label_str, const int goal_position);;
  bool evaluate(const World &state) const override;
 private:
  int goal_position_;
};

#endif //MAMCTS_TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_OTHER_GOAL_REACHED_HPP_
