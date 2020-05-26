//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_DIRECT_FRONT_H_
#define TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_DIRECT_FRONT_H_

#include "crossing_test/label_evaluator/evaluator_label_multi_agent.h"

class EvaluatorLabelInDirectFront : public EvaluatorLabelMultiAgent {
 public:
  EvaluatorLabelInDirectFront(const std::string& label_str);

 protected:
  bool evaluate_agent(const World& state, int agent_id) const override;
};

#endif  // TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_DIRECT_FRONT_H_
