// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef LTL_EVALUATOR_RULE_STATE_H_
#define LTL_EVALUATOR_RULE_STATE_H_

#include <memory>
#include <ostream>
#include <vector>

#include "ltl_evaluator/common.h"
#include "ltl_evaluator/evaluator_rule_ltl.h"

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
  RuleState(uint32_t current_state, double rule_belief, size_t violated,
            std::shared_ptr<const EvaluatorRuleLTL> automaton,
            std::vector<int> agent_ids = {});
  uint32_t current_state_;
  double rule_belief_;
  size_t violated_;
  std::shared_ptr<const EvaluatorRuleLTL> automaton_;
  std::vector<int> agent_ids_;
};

}  // namespace ltl

#endif  // LTL_EVALUATOR_RULE_STATE_H_
