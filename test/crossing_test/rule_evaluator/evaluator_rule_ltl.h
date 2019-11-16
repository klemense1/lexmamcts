//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//


#ifndef BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_
#define BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_

#include <string>
#include <set>

#include "common.h"
#include "Eigen/Core"
#include "spot/tl/parse.hh"
#include "spot/twaalgos/translate.hh"
#include "test/crossing_test/rule_evaluator/rule_state.h"

namespace modules {
namespace models {
namespace behavior {

typedef std::map<std::string, bool> EvaluationMap;

class RuleState;

class EvaluatorRuleLTL : public std::enable_shared_from_this<EvaluatorRuleLTL> {
 public:
  typedef std::shared_ptr<EvaluatorRuleLTL> EvaluatorRuleLTLSPtr;

  template<typename... T>
  static EvaluatorRuleLTLSPtr make_rule(T &&... t) {
    return EvaluatorRuleLTLSPtr(new EvaluatorRuleLTL(std::forward<T>(t)...));
  }

  RuleState make_rule_state() const;

  float evaluate(EvaluationMap const &labels, RuleState &state) const;

  float get_final_reward(const RuleState &state) const;

  void update_belief(RuleState &state) const;

  RewardPriority get_type() const;

  void set_weight(float weight);

  void set_final_reward(float final_reward);

  void set_type(RewardPriority type);

  friend std::ostream &operator<<(std::ostream &os, EvaluatorRuleLTL const &d);
 private:
  EvaluatorRuleLTL(spot::formula ltl_formula,
                   float weight,
                   RewardPriority type,
                   float init_belief = 1.0,
                   float final_reward = 0.0f);

  EvaluatorRuleLTL(std::string ltl_formula_str,
                   float weight,
                   RewardPriority type,
                   float init_belief = 1.0,
                   float final_reward = 0.0f);
  static spot::formula parse_formula(std::string ltl_formula_str);
  static bool bdd_eval(bdd cond, const std::set<int> &vars);

  float weight_;
  float final_reward_;
  spot::twa_graph_ptr aut_;
  std::set<spot::formula> alphabet_;
  spot::formula ltl_formula_;
  RewardPriority type_;
  Eigen::Matrix2d observation_prob_;
  const float init_belief_;
};
}
}
}
#endif //BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_
