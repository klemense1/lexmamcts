//
// Created by Luis Gressenbuch on 16.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_RULE_EVALUATOR_COMMON_H_
#define MAMCTS_TEST_CROSSING_TEST_RULE_EVALUATOR_COMMON_H_

namespace modules {
namespace models {
namespace behavior {

enum RewardPriority {
  SAFETY = 0, LEGAL_RULE, LEGAL_RULE_B, LEGAL_RULE_C, GOAL,
};
}
}
}

#endif //MAMCTS_TEST_CROSSING_TEST_RULE_EVALUATOR_COMMON_H_
