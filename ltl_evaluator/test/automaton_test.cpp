//
// Created by Luis Gressenbuch on 16.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gtest/gtest.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "ltl_evaluator/evaluator_rule_ltl.h"
#include "ltl_evaluator/label.h"

using namespace ltl;
using EvaluatorRuleLTLSPtr = EvaluatorRuleLTL::EvaluatorRuleLTLSPtr;

TEST(AutomatonTest, simple) {
  EvaluatorRuleLTLSPtr aut = EvaluatorRuleLTL::make_rule("G label", -1.0f, RewardPriority::SAFETY);
  EvaluationMap labels;
  labels.insert({Label("label"), true});
  RuleState state = aut->make_rule_state()[0];
  double res = state.get_automaton()->evaluate(labels, state);
  ASSERT_EQ(0.0, res);
  labels.clear();
  labels.insert({Label("label"), false});
  res = state.get_automaton()->evaluate(labels, state);
  ASSERT_EQ(-1.0, res);
}

TEST(AutomatonTest, liveness) {
  EvaluatorRuleLTLSPtr aut = EvaluatorRuleLTL::make_rule("F label", -1.0f, RewardPriority::SAFETY);
  EvaluationMap labels;
  labels.insert({Label("label"), false});
  RuleState state = aut->make_rule_state()[0];
  ASSERT_EQ(state.get_automaton()->evaluate(labels, state), 0.0);
  ASSERT_EQ(state.get_automaton()->evaluate(labels, state), 0.0);
  ASSERT_EQ(state.get_automaton()->get_final_reward(state), -1.0);
  labels.clear();
  labels.insert({Label("label"), true});
  ASSERT_EQ(state.get_automaton()->evaluate(labels, state), 0.0);
  ASSERT_EQ(state.get_automaton()->get_final_reward(state), 0.0);
}

TEST(AutomatonTest, parse_agent) {
  EvaluatorRuleLTLSPtr aut = EvaluatorRuleLTL::make_rule("G agent#0", -1.0f, RewardPriority::SAFETY);
  aut = EvaluatorRuleLTL::make_rule("G agent_1_test#0", -1.0f, RewardPriority::SAFETY);
  aut = EvaluatorRuleLTL::make_rule("G agent_1_test#0 & agent2#1 & env", -1.0f, RewardPriority::SAFETY);
  // TODO: add checks
}

TEST(AutomatonTest, agent_specific_rule_state) {
  EvaluatorRuleLTLSPtr aut = EvaluatorRuleLTL::make_rule("G (a#0 & b#1)", -1.0f, 0);
  auto rule_states = aut->make_rule_state({1,2});
  EXPECT_EQ(2, rule_states.size());
  EXPECT_EQ(1,rule_states[0].get_agent_ids()[0]);
  EXPECT_EQ(2,rule_states[0].get_agent_ids()[1]);
  EXPECT_EQ(2,rule_states[1].get_agent_ids()[0]);
  EXPECT_EQ(1,rule_states[1].get_agent_ids()[1]);

  aut = EvaluatorRuleLTL::make_rule("G a", -1.0f, 0);
  rule_states = aut->make_rule_state();
  EXPECT_EQ(1, rule_states.size());
  EXPECT_FALSE(rule_states[0].is_agent_specific());
}

TEST(AutomatonTest, agent_specific_rule_transition) {
  EvaluatorRuleLTLSPtr aut = EvaluatorRuleLTL::make_rule("G label#0", -1.0f, RewardPriority::SAFETY);
  EvaluationMap labels;
  labels.insert({Label("label", 1), true});
  RuleState state = aut->make_rule_state({1})[0];
  double res = state.get_automaton()->evaluate(labels, state);
  ASSERT_EQ(0.0, res);
  labels.clear();
  labels.insert({Label("label", 1), false});
  res = state.get_automaton()->evaluate(labels, state);
  ASSERT_EQ(-1.0, res);
}

int main(int argc, char **argv) {
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = true;
  return RUN_ALL_TESTS();
}