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
#include "spot/tl/parse.hh"
#include "spot/twaalgos/translate.hh"

namespace ltl {

typedef std::unordered_map<std::string, bool> EvaluationMap;

class RuleState;

class EvaluatorRuleLTL : public std::enable_shared_from_this<EvaluatorRuleLTL> {
 public:
  typedef std::shared_ptr<EvaluatorRuleLTL> EvaluatorRuleLTLSPtr;

  static EvaluatorRuleLTLSPtr make_rule(std::string ltl_formula_str, float weight,
                                        RulePriority priority, float init_belief = 1.0,
                                        float final_reward = 0.0f) {
    return EvaluatorRuleLTLSPtr(new EvaluatorRuleLTL(ltl_formula_str, weight, priority, init_belief, final_reward));
  }

  RuleState make_rule_state() const;

  float evaluate(EvaluationMap const &labels, RuleState &state) const;

  float get_final_reward(const RuleState &state) const;

  void update_belief(RuleState &state) const;

  RulePriority get_priority() const;

  void set_weight(float weight);

  void set_final_reward(float final_reward);

  void set_priority(RulePriority priority);

  friend std::ostream &operator<<(std::ostream &os, EvaluatorRuleLTL const &d);

 private:
  EvaluatorRuleLTL(spot::formula ltl_formula, float weight,
                   RulePriority priority, float init_belief = 1.0,
                   float final_reward = 0.0f);

  EvaluatorRuleLTL(std::string ltl_formula_str, float weight,
                   RulePriority priority, float init_belief = 1.0,
                   float final_reward = 0.0f);
  static spot::formula parse_formula(std::string ltl_formula_str);
  static bool bdd_eval(bdd cond, const std::set<int> &vars);

  float weight_;
  float final_reward_;
  spot::twa_graph_ptr aut_;
  std::set<spot::formula> alphabet_;
  spot::formula ltl_formula_;
  RulePriority priority_;
  Eigen::Matrix2d observation_prob_;
  const float init_belief_;
};
}  // namespace ltl
#endif  // LTL_EVALUATOR_EVALUATOR_RULE_LTL_H_
