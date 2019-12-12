//
// Created by luis on 07.10.19.
//

#include "evaluator_rule_ltl.h"
#include <numeric>
#include <regex>
#include "glog/logging.h"
#include "ltl_evaluator/label.h"
#include "spot/tl/apcollect.hh"
#include "spot/tl/print.hh"
#include "spot/twa/bddprint.hh"

namespace ltl {
EvaluatorRuleLTL::EvaluatorRuleLTL(const std::string& ltl_formula_str, float weight,
                                   RulePriority priority, float init_belief,
                                   float final_reward)
    : weight_(weight),
      final_reward_(final_reward),
      priority_(priority),
      init_belief_(init_belief),
      is_agent_specific_(false) {
  assert(init_belief <= 1.0 && init_belief >= 0.0);

  const std::string agent_free_formula = parse_agents(ltl_formula_str);
  ltl_formula_ = parse_formula(agent_free_formula);
  spot::translator trans;
  trans.set_pref(spot::postprocessor::Deterministic);
  trans.set_type(spot::postprocessor::BA);
  aut_ = trans.run(ltl_formula_);
  observation_prob_ << 0.9, 0.1, 0.5, 0.5;
}
std::string EvaluatorRuleLTL::parse_agents(const std::string &ltl_formula_str) {
  std::string remaining = ltl_formula_str;
  std::string agent_free_formula;
  std::regex r("([[:lower:][:digit:]_]+)(#([[:digit:]])+)?");
  std::smatch sm;
  while(std::regex_search(remaining, sm, r)) {
    std::string ap_name = sm[1];
    for(const auto &a : sm) {
      DVLOG(2) << a;
    }
    int agent_id_placeholder = -1;
    if(sm[3] != "") {
      agent_id_placeholder = std::stoi(sm[3]);
      is_agent_specific_ = true;
    }
    agent_free_formula += sm.prefix();
    agent_free_formula += ap_name;
    ap_alphabet_.push_back({ap_name, spot::formula::ap(ap_name), agent_id_placeholder, is_agent_specific_});
    remaining = sm.suffix();
  }
  agent_free_formula += remaining;
  VLOG(1) << "Cleaned formula: " << agent_free_formula;
  return agent_free_formula;
}

std::vector<RuleState> EvaluatorRuleLTL::make_rule_state(std::vector<int> agent_ids) const {
  assert(is_agent_specific_ == !agent_ids.empty());
  int num_other_agents = std::max_element(ap_alphabet_.begin(), ap_alphabet_.end(), [](const APContainer &a, const APContainer &b) {
    return (a.id_idx_ < b.id_idx_);
  })->id_idx_ + 1;
  num_other_agents = std::max(num_other_agents, 0);
  assert(agent_ids.size() >= num_other_agents);
  std::vector<RuleState> l;
  std::vector<std::vector<int>> permutations =
      all_k_permutations(agent_ids, num_other_agents);
  for(const auto &perm : permutations) {
    l.push_back(RuleState(aut_->get_init_state_number(), init_belief_, 0,
                          shared_from_this(), perm));
  }
  return l;
}
std::vector<std::vector<int>> EvaluatorRuleLTL::all_k_permutations(
    const std::vector<int> &values, int k) const {
  std::vector<std::vector<int>> permutations;
  std::vector<int> value_permutation(k, 0);
  std::vector<int> idx_permutation(values.size());
  std::iota(idx_permutation.begin(), idx_permutation.end(),0);
  // Create all k-permutations of the agent ids
  do
  {
    for (int i = 0; i < k; i++)
    {
      value_permutation[i] = values[idx_permutation[i]];
    }
    permutations.emplace_back(value_permutation);
    std::reverse(idx_permutation.begin()+k, idx_permutation.end());
  } while (std::next_permutation(idx_permutation.begin(), idx_permutation.end()));
  return permutations;
}

float EvaluatorRuleLTL::evaluate(const EvaluationMap &labels,
                                 RuleState &state) const {
  std::set<int> bddvars = std::set<int>();
  spot::bdd_dict_ptr bddDictPtr = aut_->get_dict();
  // Self looping behavior
  uint32_t next_state = state.current_state_;
  for (const auto &ap : ap_alphabet_) {
    Label label;
    if(ap.is_agent_specific) {
      label = Label(ap.ap_str_, state.get_agent_ids()[ap.id_idx_]);
    } else {
      label = Label(ap.ap_str_);
    }
    auto it = labels.find(label);
    if(it == labels.end()) {
      // Rule is undecided
      VLOG(2) << "Rule undecided! Missing label: " << ap.ap_str_ << "_" << state.get_agent_ids()[ap.id_idx_];
      return 0.0f;
    }
    if (it->second) {
      int bdd_var = bddDictPtr->has_registered_proposition(ap.ap_, aut_);
      bddvars.insert(bdd_var);
    }
  }
  bool transition_found = false;
  for (const auto &transition : aut_->out(state.current_state_)) {
    if (EvaluatorRuleLTL::evaluate_bdd(transition.cond, bddvars)) {
      next_state = transition.dst;
      transition_found = true;
      break;
    }
  }
  state.current_state_ = next_state;
  state.violated_ = transition_found ? state.violated_ : state.violated_ + 1;

  return !transition_found ? state.rule_belief_ * weight_ : 0.0f;
}

bool EvaluatorRuleLTL::evaluate_bdd(bdd cond, const std::set<int> &vars) {
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
  os << "\", weight: " << d.weight_ << ", priority: " << d.priority_;
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
bool EvaluatorRuleLTL::is_agent_specific() const { return is_agent_specific_; }
}