//
// Created by Luis Gressenbuch on 05.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gtest/gtest.h"
#include "glog/logging.h"
#include "ltl_evaluator/evaluator_rule_ltl.h"

using namespace ltl;
using EvaluatorRuleLTLSPtr = EvaluatorRuleLTL::EvaluatorRuleLTLSPtr;

class ZipperMergeFormula : public testing::TestWithParam<std::string> {};


/// Return true on violation
/// \param rs Current rule state
/// \param labels Input labels
/// \return violated?
inline bool check_violation(RuleState *rs, const EvaluationMap &labels) {
  float res = rs->get_automaton()->evaluate(labels, *rs);
  return (res != 0.0);
}

inline bool check_final(const RuleState &rs) {
  float res = rs.get_automaton()->get_final_reward(rs);
  return (res != 0.0);
}

// Two agents at merging point
TEST_P(ZipperMergeFormula, collision) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = true;
  labels["wl"] = true;
  labels["mp_r"] = true;
  labels["wr"] = true;
  EXPECT_TRUE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Two agents from left
TEST_P(ZipperMergeFormula, false_alternation_l) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_TRUE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Two agents from right
TEST_P(ZipperMergeFormula, false_alternation_r) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_TRUE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Correct alternation, beginning left
TEST_P(ZipperMergeFormula, true_alternation_l) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Correct alternation, beginning right
TEST_P(ZipperMergeFormula, true_alternation_r) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// No waiting agent on left
TEST_P(ZipperMergeFormula, trivial_merge_r) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = false;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = false;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = false;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// No waiting agent on right
TEST_P(ZipperMergeFormula, trivial_merge_l) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Agent arriving on the right, while other agent is at merging point
TEST_P(ZipperMergeFormula, intermittent_arrive_r) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = false;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Agent arriving on the right, while other agent is at merging point
TEST_P(ZipperMergeFormula, fail_intermittent_arrive_r) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = true;
  labels["wr"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = true;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = true;
  EXPECT_TRUE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = true;
  labels["wl"] = false;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["mp_l"] = false;
  labels["mp_r"] = false;
  labels["wl"] = false;
  labels["wr"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

INSTANTIATE_TEST_CASE_P(ZipperMergeFormulaTest, ZipperMergeFormula,
    testing::Values(
    "G((mp_l & (X !mp_l) & wr) -> X(!mp_l W mp_r)) & G((mp_r & (X !mp_r) & wl) -> X(!mp_r W mp_l)) & G!(mp_l & mp_r) & G(wl & !wr & !mp_l & !mp_r-> X(!mp_r W mp_l)) & G(wr & !wl & !mp_l & !mp_r -> X(!mp_l W mp_r))",
    "G((mp_l & wr) -> X(!mp_l U mp_r)) & G!(mp_l & mp_r) & G((mp_r & wl) -> X(!mp_r U mp_l)) & G(wl & !wr -> X(!mp_r U mp_l)) & G(wr & !wl -> X(!mp_l U mp_r))"));

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_v = 2;



  return RUN_ALL_TESTS();
}