// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_AT_POSITION_H_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_AT_POSITION_H_

#include <string>
#include <utility>
#include <vector>

#include "gridworld/common.hpp"
#include "gridworld/label_evaluator/evaluator_label_base.h"

class EvaluatorLabelAtPosition : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelAtPosition(const std::string &label_str, const int position);
  std::vector<std::pair<ltl::Label, bool>> evaluate(
      const World &state) const override;
  void set_position(int position);

 private:
  int position_;
};

#endif  // MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_AT_POSITION_H_
