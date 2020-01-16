//
// Created by Luis Gressenbuch on 16.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef LTL_COMMON_H_
#define LTL_COMMON_H_

namespace ltl {

typedef unsigned int RulePriority;
enum RewardPriority {
  SAFETY = 0,
  LEGAL_RULE,
  LEGAL_RULE_B,
  LEGAL_RULE_C,
  GOAL,
};
}  // namespace ltl

#endif  // LTL_COMMON_H_
