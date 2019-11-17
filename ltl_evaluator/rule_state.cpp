//
// Created by Luis Gressenbuch on 16.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "ltl_evaluator/rule_state.h"

namespace modules {
namespace models {
namespace behavior {
uint32_t RuleState::get_current_state() const {
  return current_state_;
}
RewardPriority RuleState::get_type() const {
  return automaton_->get_type();
}
double RuleState::get_rule_belief() const {
  return rule_belief_;
}
size_t RuleState::get_violation_count() const {
  return violated_;
}
void RuleState::reset_violations() {
  violated_ = 0;
}
const std::shared_ptr<const EvaluatorRuleLTL> &RuleState::get_automaton() const {
  return automaton_;
}
std::ostream &operator<<(std::ostream &os, const RuleState &state) {
  os << "current_state_: " << state.current_state_ << " rule_belief_: " << state.rule_belief_ << " violated_: "
     << state.violated_ << " automaton_: " << state.automaton_;
  return os;
}
RuleState::RuleState(uint32_t current_state,
                     double rule_belief,
                     size_t violated,
                     std::shared_ptr<const EvaluatorRuleLTL> automaton)
    : current_state_(current_state), rule_belief_(rule_belief), violated_(violated), automaton_(automaton) {}
}
}
}