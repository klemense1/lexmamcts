//
// Created by luis on 07.10.19.
//

#include "glog/logging.h"
#include "spot/twa/bddprint.hh"
#include "spot/tl/print.hh"
#include "evaluator_rule_ltl.hpp"

namespace modules {
namespace models {
namespace behavior {
EvaluatorRuleLTL::EvaluatorRuleLTL(spot::formula ltl_formula, float weight, RewardPriority type, float init_belief,
                                   float final_reward) :
        weight_(weight), final_reward_(final_reward), ltl_formula_(ltl_formula), type_(type), violated_(false) {

  assert(init_belief <= 1.0 && init_belief >= 0.0);
  spot::translator trans;
  if (ltl_formula.is_syntactic_safety()) {
    trans.set_type(spot::postprocessor::Monitor);
    trans.set_pref(spot::postprocessor::Deterministic);
  } else {
    trans.set_type(spot::postprocessor::BA);
    trans.set_pref(spot::postprocessor::Complete);
  }
  aut = trans.run(ltl_formula);
  ltl_formula.traverse([this](spot::formula f) {
      if (f.is(spot::op::ap)) {
        this->alphabet.insert(f.ap_name());
        return true;
      }
      return false;
  });
  reset_state();
  rule_belief_ << init_belief, 1.0 - init_belief;
  observation_prob_ << 0.9, 0.1, 0.5, 0.5;
}

EvaluatorRuleLTL::EvaluatorRuleLTL(std::string ltl_formula_str, float weight, RewardPriority type, float init_belief,
                                   float final_reward) :
        EvaluatorRuleLTL(spot::parse_infix_psl(ltl_formula_str).f, weight, type, init_belief, final_reward) {}

float EvaluatorRuleLTL::evaluate(EvaluationMap &labels) {
  std::set<int> bddvars = std::set<int>();
  spot::bdd_dict_ptr bddDictPtr = aut->get_dict();
  // Self looping behavior
  uint32_t next_state = current_state;
  for (const auto ap_str : alphabet) {
    // Ensure the the label is decided
    assert(labels.find(ap_str) != labels.end());
    if (labels[ap_str]) {
      int bdd_var = bddDictPtr->has_registered_proposition(spot::formula::ap(ap_str), aut);
      // Label is in the alphabet so it should be assigned
      assert(bdd_var >= 0);
      bddvars.insert(bdd_var);
    }
  }
  bool transition_found = false;
  for (auto transition :  aut->out(current_state)) {
    if (EvaluatorRuleLTL::bdd_eval(transition.cond, bddvars)) {
      //std::cout << std::endl << "  edge(" << current_state << " -> " << transition.dst << ")\n    label = ";
      //spot::bdd_print_formula(std::cout, aut->get_dict(), transition.cond);
      next_state = transition.dst;
      transition_found = true;
      break;
    }
  }
  current_state = next_state;
  violated_ = violated_ || !transition_found;
  return !transition_found ? rule_belief_(0) * weight_ : 0.0f;
}

void EvaluatorRuleLTL::reset_state() {
  current_state = aut->get_init_state_number();
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

float EvaluatorRuleLTL::get_final_reward() const {
  float penalty = final_reward_;
  // Check if formula has liveness property and is in accepting state
  if (!ltl_formula_.is_syntactic_safety() && !aut->state_is_accepting(current_state)) {
    penalty = weight_;
  }
  return rule_belief_(0) * penalty;
}

RewardPriority EvaluatorRuleLTL::get_type() const {
  return type_;
}

std::ostream &operator<<(std::ostream &os, EvaluatorRuleLTL const &d) {
  os << "\"";
  spot::print_psl(os, d.ltl_formula_);
  os << "\", weight: " << d.weight_;
  os << ", priority: " << d.get_type();
  os << ", belief state: " << d.rule_belief_.transpose();
  return os;
}

void EvaluatorRuleLTL::reset_violation() {
  violated_ = false;
}

void EvaluatorRuleLTL::update_belief() {
  if (!ltl_formula_.is_syntactic_safety() && !aut->state_is_accepting(current_state)) {
    violated_ = true;
  }
  if (violated_) {
    int observation = violated_ ? 1 : 0;
    double eta = 1.0 / (observation_prob_.col(observation).transpose() * rule_belief_)(0);
    rule_belief_ = eta * observation_prob_.col(observation).cwiseProduct(rule_belief_);
  }
}
}
}
}