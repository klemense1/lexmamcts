//
// Created by luis on 07.10.19.
//

#ifndef BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_
#define BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_

#include <string>
#include <set>

#include "spot/tl/parse.hh"
#include "spot/twaalgos/translate.hh"

namespace modules {
namespace models {
namespace behavior {

typedef std::map<std::string, bool> EvaluationMap;

enum RewardPriority {
  SAFETY = 0,
  GOAL = 0,
  TIME,
  EFFICIENCY,
};

class EvaluatorRuleLTL {
 public:
  EvaluatorRuleLTL(spot::formula ltl_formula, float weight, RewardPriority type);
  EvaluatorRuleLTL(std::string ltl_formula_str, float weight, RewardPriority type);
  //EvaluatorRuleLTL(const EvaluatorRuleLTL &evaluator_rule_ltl) = default;
  float evaluate(EvaluationMap &labels);
  float final_reward();
  void reset_state();
 private:
  static bool bdd_eval(bdd cond, const std::set<int> &vars);
  float weight_;
  uint32_t current_state;
  spot::twa_graph_ptr aut;
  std::set<std::string> alphabet;
  spot::formula ltl_formula_;
  RewardPriority type_;
 public:
  RewardPriority get_type() const;
};

}
}
}
#endif //BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_
