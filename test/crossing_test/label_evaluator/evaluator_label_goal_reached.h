//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_GOAL_REACHED_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_GOAL_REACHED_HPP_

#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelGoalReached : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelGoalReached(const std::string &label_str, const int goal_position);;
  std::vector<std::pair<ltl::Label, bool>> evaluate(
      const World &state) const override;

 private:
  int goal_position_;
};

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_GOAL_REACHED_HPP_
