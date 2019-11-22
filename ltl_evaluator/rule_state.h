//
// Created by Luis Gressenbuch on 16.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_RULE_STATE_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_RULE_STATE_H_

#include <ostream>
#include "common.h"
#include "evaluator_rule_ltl.h"

namespace modules {
namespace models {
namespace behavior {

class EvaluatorRuleLTL;

class RuleState {
 public:
  friend class EvaluatorRuleLTL;
  uint32_t get_current_state() const;
  RulePriority get_type() const;
  double get_rule_belief() const;
  size_t get_violation_count() const;
  void reset_violations();
  const std::shared_ptr<const EvaluatorRuleLTL> &get_automaton() const;
  friend std::ostream &operator<<(std::ostream &os, const RuleState &state);
 private:
  RuleState(uint32_t current_state,
            double rule_belief,
            size_t violated,
            std::shared_ptr<const EvaluatorRuleLTL> automaton);
  uint32_t current_state_;
  double rule_belief_;
  size_t violated_;
  std::shared_ptr<const EvaluatorRuleLTL> automaton_;
};

}
}
}

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_RULE_STATE_H_
