//
// Created by luis on 07.10.19.
//

#include "evaluator_rule_ltl.h"
#include "glog/logging.h"
#include "spot/tl/apcollect.hh"
#include "spot/tl/print.hh"
#include "spot/twa/bddprint.hh"

namespace ltl {
EvaluatorRuleLTL::EvaluatorRuleLTL(spot::formula ltl_formula, float weight,
                                   RulePriority priority, float init_belief,
                                   float final_reward)
    : weight_(weight),
      final_reward_(final_reward),
      ltl_formula_(ltl_formula),
      priority_(priority),
      init_belief_(init_belief) {
  assert(init_belief <= 1.0 && init_belief >= 0.0);
  spot::translator trans;
  if (ltl_formula.is_syntactic_safety()) {
    trans.set_type(spot::postprocessor::Monitor);
    trans.set_pref(spot::postprocessor::Deterministic);
  } else {
    trans.set_type(spot::postprocessor::BA);
    trans.set_pref(spot::postprocessor::Complete);
  }
  aut_ = trans.run(ltl_formula);
  spot::atomic_prop_collect(ltl_formula_, &alphabet_);
  observation_prob_ << 0.9, 0.1, 0.5, 0.5;
}

EvaluatorRuleLTL::EvaluatorRuleLTL(std::string ltl_formula_str, float weight,
                                   RulePriority priority, float init_belief,
                                   float final_reward)
    : EvaluatorRuleLTL(parse_formula(ltl_formula_str), weight, priority,
                       init_belief, final_reward) {}

RuleState EvaluatorRuleLTL::make_rule_state() const {
  return RuleState(aut_->get_init_state_number(), init_belief_, 0,
                   shared_from_this());
}

float EvaluatorRuleLTL::evaluate(EvaluationMap const &labels,
                                 RuleState &state) const {
  std::set<int> bddvars = std::set<int>();
  spot::bdd_dict_ptr bddDictPtr = aut_->get_dict();
  // Self looping behavior
  uint32_t next_state = state.current_state_;
  for (const auto ap : alphabet_) {
    // Ensure the the label is decided
    assert(labels.find(ap.ap_name()) != labels.end());
    if (labels.at(ap.ap_name())) {
      int bdd_var = bddDictPtr->has_registered_proposition(ap, aut_);
      // Label is in the alphabet_ so it should be assigned
      assert(bdd_var >= 0);
      bddvars.insert(bdd_var);
    }
  }
  bool transition_found = false;
  for (auto transition : aut_->out(state.current_state_)) {
    if (EvaluatorRuleLTL::bdd_eval(transition.cond, bddvars)) {
      next_state = transition.dst;
      transition_found = true;
      break;
    }
  }
  state.current_state_ = next_state;
  state.violated_ = transition_found ? state.violated_ : state.violated_ + 1;

  return !transition_found ? state.rule_belief_ * weight_ : 0.0f;
}

bool EvaluatorRuleLTL::bdd_eval(bdd cond, const std::set<int> &vars) {
  bdd bdd_node = cond;
  while (bdd_node != bddtrue && bdd_node != bddfalse) {
    if (vars.find(bdd_var(bdd_node)) != vars.end()) {
      bdd_node = bdd_high(bdd_node);
    } else {
      bdd_node = bdd_low(bdd_node);
    }
  }
  return bdd_node == bddtrue;
}

float EvaluatorRuleLTL::get_final_reward(const RuleState &state) const {
  float penalty = final_reward_;
  // Check if formula has liveness property and is in accepting state
  if (!ltl_formula_.is_syntactic_safety() &&
      !aut_->state_is_accepting(state.current_state_)) {
    penalty = weight_;
  }
  return state.rule_belief_ * penalty;
}

std::ostream &operator<<(std::ostream &os, EvaluatorRuleLTL const &d) {
  os << "\"";
  spot::print_psl(os, d.ltl_formula_);
  os << "\", weight: " << d.weight_;
  return os;
}

void EvaluatorRuleLTL::update_belief(RuleState &state) const {
  Eigen::Vector2d belief_v(state.rule_belief_, 1.0 - state.rule_belief_);
  if (!ltl_formula_.is_syntactic_safety() &&
      !aut_->state_is_accepting(state.current_state_)) {
    ++state.violated_;
  }
  if (state.violated_ > 0) {
    int observation = 1;
    double eta =
        1.0 / (observation_prob_.col(observation).transpose() * belief_v)(0);
    belief_v = eta * observation_prob_.col(observation).cwiseProduct(belief_v);
    state.rule_belief_ = belief_v(0);
  }
}

spot::formula EvaluatorRuleLTL::parse_formula(std::string ltl_formula_str) {
  spot::parsed_formula pf = spot::parse_infix_psl(ltl_formula_str);
  if (pf.errors.size()) {
    pf.format_errors(LOG(FATAL));
  }
  return pf.f;
}
void EvaluatorRuleLTL::set_weight(float weight) { weight_ = weight; }
void EvaluatorRuleLTL::set_final_reward(float final_reward) {
  final_reward_ = final_reward;
}
void EvaluatorRuleLTL::set_priority(RulePriority priority) {
  priority_ = priority;
}
RulePriority EvaluatorRuleLTL::get_priority() const { return priority_; }
}