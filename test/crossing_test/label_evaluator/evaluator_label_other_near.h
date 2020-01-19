//
// Created by luis on 15.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_

#include <string>
#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelOtherNear : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelOtherNear(const std::string &label_str);
  std::vector<std::pair<ltl::Label, bool>> evaluate(
      const World &state) const override;
};

#endif  // MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_OTHER_NEAR_HPP_
