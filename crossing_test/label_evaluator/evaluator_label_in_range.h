//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_RANGE_H_
#define TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_RANGE_H_

#include "crossing_test/label_evaluator/evaluator_label_multi_agent.h"
class EvaluatorLabelInRange : public EvaluatorLabelMultiAgent {
 public:
  EvaluatorLabelInRange(const std::string& label_str, int range_start,
                        int range_end);

 protected:
  bool evaluate_agent(const World& state, int agent_id) const override;

 private:
  int range_start_;
  int range_end_;
};

#endif  // TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_IN_RANGE_H_
