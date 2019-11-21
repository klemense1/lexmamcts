//
// Created by Luis Gressenbuch on 16.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gtest/gtest.h"
#include "ltl_evaluator/evaluator_rule_ltl.h"

using namespace modules::models::behavior;
using EvaluatorRuleLTLSPtr = EvaluatorRuleLTL::EvaluatorRuleLTLSPtr;

TEST(AutomatonTest, simple) {
  EvaluatorRuleLTLSPtr aut = EvaluatorRuleLTL::make_rule("G label", -1.0f, RewardPriority::SAFETY);
  EvaluationMap labels;
  labels.insert({"label", true});
  RuleState state = aut->make_rule_state();
  double res = state.get_automaton()->evaluate(labels, state);
  ASSERT_EQ(res, 0.0);
  labels.insert({"label", false});
  res = state.get_automaton()->evaluate(labels, state);
  ASSERT_EQ(res, -1.0);
}

TEST(AutomatonTest, liveness) {
  EvaluatorRuleLTLSPtr aut = EvaluatorRuleLTL::make_rule("F label", -1.0f, RewardPriority::SAFETY);
  EvaluationMap labels;
  labels.insert({"label", false});
  RuleState state = aut->make_rule_state();
  ASSERT_EQ(state.get_automaton()->evaluate(labels, state), 0.0);
  ASSERT_EQ(state.get_automaton()->evaluate(labels, state), 0.0);
  ASSERT_EQ(state.get_automaton()->get_final_reward(state), -1.0);
  labels.insert({"label", true});
  ASSERT_EQ(state.get_automaton()->evaluate(labels, state), 0.0);
  ASSERT_EQ(state.get_automaton()->get_final_reward(state), 0.0);
}