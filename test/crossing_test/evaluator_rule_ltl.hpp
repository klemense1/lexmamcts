//
// Created by luis on 07.10.19.
//

#ifndef BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_
#define BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_

#include <string>
#include <set>

#include "Eigen/Core"
#include "spot/tl/parse.hh"
#include "spot/twaalgos/translate.hh"

namespace modules {
namespace models {
namespace behavior {

typedef std::map<std::string, bool> EvaluationMap;

enum RewardPriority {
  SAFETY = 0,
  LEGAL_RULE = 1,
  LEGAL_RULE_B = 2,
  GOAL = 1,
  TIME = 2,
  EFFICIENCY = 3,
};

class EvaluatorRuleLTL {
public:
  EvaluatorRuleLTL(spot::formula ltl_formula, float weight, RewardPriority type, float init_belief = 1.0,
                   float final_reward = 0.0f);

  EvaluatorRuleLTL(std::string ltl_formula_str, float weight, RewardPriority type, float init_belief = 1.0,
                   float final_reward = 0.0f);

  float evaluate(EvaluationMap &labels);

  float get_final_reward() const;

  void reset_state();

  RewardPriority get_type() const;

  void reset_violation();

  void update_belief();

  friend std::ostream &operator<<(std::ostream &os, EvaluatorRuleLTL const &d);

private:
  static spot::formula parse_formula(std::string ltl_formula_str);
  static bool bdd_eval(bdd cond, const std::set<int> &vars);

  float weight_;
  float final_reward_;
  uint32_t current_state_;
  spot::twa_graph_ptr aut_;
  std::set<spot::formula> alphabet_;
  spot::formula ltl_formula_;
  RewardPriority type_;
  Eigen::Vector2f rule_belief_;
  Eigen::Matrix2f observation_prob_;
  bool violated_;
};

}
}
}
#endif //BARK_MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_EVALUATOR_RULE_LTL_HPP_
