//
// Created by luis on 07.10.19.
//

#include "evaluator_rule_ltl.hpp"

namespace modules {
namespace models {
namespace behavior {
EvaluatorRuleLTL::EvaluatorRuleLTL(spot::formula ltl_formula, float weight, RewardType type) :
    weight_(weight), ltl_formula_(ltl_formula), type_(type) {

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
}

EvaluatorRuleLTL::EvaluatorRuleLTL(std::string ltl_formula_str, float weight, RewardType type) :
    EvaluatorRuleLTL(spot::parse_infix_psl(ltl_formula_str).f, weight, type) {}

float EvaluatorRuleLTL::evaluate(EvaluationMap &labels) {
  std::set<int> bddvars = std::set<int>();
  spot::bdd_dict_ptr bddDictPtr = aut->get_dict();
  // Self looping behavior
  spot::state *next_state = current_state;
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
  for (auto transition :  aut->succ(current_state)) {
    if (EvaluatorRuleLTL::bdd_eval(transition->cond(), bddvars)) {
      next_state = transition->dst()->clone();
      transition_found = true;
      break;
    }
  }
  current_state = next_state;
  return !transition_found ? weight_ : 0.0f;
}

void EvaluatorRuleLTL::reset_state() {
  current_state = aut->get_init_state()->clone();
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
float EvaluatorRuleLTL::final_reward() {
  float penalty = 0.0f;
  // Check if formula has liveness property and is in accepting state
  if (!ltl_formula_.is_syntactic_safety() && !aut->state_is_accepting(current_state)) {
    penalty = weight_;
  }
  return penalty;
}
RewardType EvaluatorRuleLTL::get_type() const {
  return type_;
}
}
}
}