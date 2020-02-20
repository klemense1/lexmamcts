//
// Created by luis on 15.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_AT_POSITION_H_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_AT_POSITION_H_

#include <string>
#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelAtPosition : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelAtPosition(const std::string &label_str, const int position);
  std::vector<std::pair<ltl::Label, bool>> evaluate(
      const World &state) const override;
  void set_position(int position);

 private:
  int position_;
};

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_AT_POSITION_H_
