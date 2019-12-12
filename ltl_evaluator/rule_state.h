//
// Created by Luis Gressenbuch on 16.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_RULE_STATE_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_RULE_STATE_H_

#include <ostream>
#include <memory>
#include <vector>
#include "common.h"
#include "evaluator_rule_ltl.h"

namespace ltl {

class EvaluatorRuleLTL;

class RuleState {
 public:
  friend class EvaluatorRuleLTL;
  uint32_t get_current_state() const;
  RulePriority get_priority() const;
  double get_rule_belief() const;
  size_t get_violation_count() const;
  void reset_violations();
  const std::shared_ptr<const EvaluatorRuleLTL> &get_automaton() const;
  bool is_agent_specific() const;
  const std::vector<int> &get_agent_ids() const;
  friend std::ostream &operator<<(std::ostream &os, const RuleState &state);
 private:
  RuleState(uint32_t current_state,
            double rule_belief,
            size_t violated,
            std::shared_ptr<const EvaluatorRuleLTL> automaton, std::vector<int> agent_ids = {});
  uint32_t current_state_;
  double rule_belief_;
  size_t violated_;
  std::shared_ptr<const EvaluatorRuleLTL> automaton_;
  std::vector<int> agent_ids_;
};

}

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_RULE_STATE_H_
