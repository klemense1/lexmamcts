//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_

#include "test/crossing_test/common.hpp"
#include "ltl_evaluator/evaluator_label_base.h"

class EvaluatorLabelCollision : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelCollision(const std::string &label_str, const int crossing_point);;
  bool evaluate(const World &state) const override;
 private:
  int crossing_point_;
};

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
