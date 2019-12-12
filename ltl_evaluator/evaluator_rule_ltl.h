//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef LTL_EVALUATOR_EVALUATOR_RULE_LTL_H_
#define LTL_EVALUATOR_EVALUATOR_RULE_LTL_H_

#include <set>
#include <string>
#include <unordered_map>

#include "Eigen/Core"
#include "common.h"
#include "ltl_evaluator/rule_state.h"
#include "ltl_evaluator/label.h"
#include "spot/tl/parse.hh"
#include "spot/twaalgos/translate.hh"

namespace ltl {

typedef std::unordered_map<Label, bool, LabelHash> EvaluationMap;

class RuleState;

class EvaluatorRuleLTL : public std::enable_shared_from_this<EvaluatorRuleLTL> {
 public:
  typedef std::shared_ptr<EvaluatorRuleLTL> EvaluatorRuleLTLSPtr;

  static EvaluatorRuleLTLSPtr make_rule(std::string ltl_formula_str, float weight,
                                        RulePriority priority, float init_belief = 1.0,
                                        float final_reward = 0.0f) {
    return EvaluatorRuleLTLSPtr(new EvaluatorRuleLTL(ltl_formula_str, weight, priority, init_belief, final_reward));
  }

  std::vector<RuleState> make_rule_state(std::vector<int> agent_ids = {}) const;

  float evaluate(EvaluationMap const &labels, RuleState &state) const;

  float get_final_reward(const RuleState &state) const;

  void update_belief(RuleState &state) const;

  RulePriority get_priority() const;

  void set_weight(float weight);

  void set_final_reward(float final_reward);

  void set_priority(RulePriority priority);

  bool is_agent_specific() const;

  friend std::ostream &operator<<(std::ostream &os, EvaluatorRuleLTL const &d);

 private:
  EvaluatorRuleLTL(const std::string& ltl_formula_str, float weight,
                   RulePriority priority, float init_belief = 1.0,
                   float final_reward = 0.0f);
  static spot::formula parse_formula(std::string ltl_formula_str);
  static bool evaluate_bdd(bdd cond, const std::set<int> &vars);

  std::string parse_agents(const std::string &ltl_formula_str);

  struct APContainer {
    std::string ap_str_;
    spot::formula ap_;
    int id_idx_;
    bool is_agent_specific;
  };

  float weight_;
  float final_reward_;
  spot::twa_graph_ptr aut_;
  std::set<spot::formula> alphabet_;
  spot::formula ltl_formula_;
  RulePriority priority_;
  Eigen::Matrix2d observation_prob_;
  const float init_belief_;
  std::vector<APContainer> ap_alphabet_;
  bool is_agent_specific_;
  std::vector<std::vector<int>> all_k_permutations(const std::vector<int> &values,
                                      int k) const;
};
}  // namespace ltl
#endif  // LTL_EVALUATOR_EVALUATOR_RULE_LTL_H_
