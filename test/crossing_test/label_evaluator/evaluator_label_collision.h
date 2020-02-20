//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_

#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelCollision : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelCollision(const std::string &label_str, const int crossing_point);
  std::vector<std::pair<ltl::Label, bool>> evaluate(
      const World &state) const override;

 private:
  int crossing_point_;
};

bool check_collision(const AgentState& a, const AgentState& b);

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
