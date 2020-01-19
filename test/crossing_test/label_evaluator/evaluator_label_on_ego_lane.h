//
// Created by Luis Gressenbuch on 17.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_ON_EGO_LANE_H_
#define TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_ON_EGO_LANE_H_

#include "test/crossing_test/label_evaluator/evaluator_label_multi_agent.h"
class EvaluatorLabelOnEgoLane : public EvaluatorLabelMultiAgent {
 public:
  EvaluatorLabelOnEgoLane(const std::string& label_str);

 protected:
  bool evaluate_agent(const World& state, int agent_id) const override;
};

#endif  // TEST_CROSSING_TEST_LABEL_EVALUATOR_EVALUATOR_LABEL_ON_EGO_LANE_H_
