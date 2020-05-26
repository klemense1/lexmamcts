// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_

#include <string>
#include <utility>
#include <vector>

#include "gridworld/common.hpp"
#include "gridworld/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelCollision : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelCollision(const std::string& label_str,
                          const int merging_point);
  std::vector<std::pair<ltl::Label, bool>> evaluate(
      const World& state) const override;

 private:
  int merging_point_;
};

bool check_collision(const AgentState& a, const AgentState& b);

#endif  // MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_COLLISION_HPP_
