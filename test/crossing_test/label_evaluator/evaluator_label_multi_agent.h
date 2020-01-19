//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_MULTI_AGENT_H_
#define TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_MULTI_AGENT_H_

#include <string>
#include "ltl/evaluator_label_base.h"
#include "test/crossing_test/common.hpp"

class EvaluatorLabelMultiAgent : public EvaluatorLabelBase<World> {
 public:
  EvaluatorLabelMultiAgent(const std::string& label_str);
  std::vector<std::pair<ltl::Label, bool>> evaluate(
      const World& state) const override;

 protected:
  virtual bool evaluate_agent(const World& state, int agent_id) const = 0;
};

#endif  // TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_MULTI_AGENT_H_
