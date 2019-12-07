//
// Created by Luis Gressenbuch on 05.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "gtest/gtest.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
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
  labels["w_e"] = false;
  labels["mp_e"] = true;
  labels["w_o"] = false;
  labels["mp_o"] = true;
  labels["mp_oe"] = false;
  EXPECT_TRUE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Ego violated
TEST_P(ZipperMergeFormula, false_alternation_l) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["w_e"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = true;
  labels["w_o"] = true;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["w_o"] = true;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["w_o"] = true;
  labels["mp_o"] = false;
  EXPECT_TRUE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Correct alternation, beginning ego lane
TEST_P(ZipperMergeFormula, true_alternation_l) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["w_e"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = true;
  labels["w_o"] = true;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// No waiting agent on other lane
TEST_P(ZipperMergeFormula, trivial_merge_r) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// On tie, w.l.g other should go first
// We are first before the merge, but have not seen any other agent passing the merge
// Should be violated if we go first
TEST_P(ZipperMergeFormula, tie_violated) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["w_e"] = true;
  labels["w_o"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_TRUE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// On tie, w.l.g other should go first
// We are first before the merge, but have not seen any other agent passing the merge
// Should be ok if other goes first
TEST_P(ZipperMergeFormula, tie) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["w_e"] = true;
  labels["w_o"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Agent arriving on the other lane, while other agent on ego lane is at merging point
TEST_P(ZipperMergeFormula, intermittent_arrive) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = true;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = true;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = true;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = false;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

// Agent arriving on the other lane, while other agent on ego lane is at merging point
TEST_P(ZipperMergeFormula, intermittent_arrive_violated) {
  EvaluatorRuleLTLSPtr rule = EvaluatorRuleLTL::make_rule(GetParam(), -1.0, 0);
  auto rs = rule->make_rule_state();
  EvaluationMap labels;
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = true;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = false;
  labels["mp_e"] = false;
  labels["mp_oe"] = true;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = true;
  labels["w_o"] = true;
  labels["mp_e"] = false;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_FALSE(check_violation(&rs, labels));
  labels["w_e"] = false;
  labels["w_o"] = true;
  labels["mp_e"] = true;
  labels["mp_oe"] = false;
  labels["mp_o"] = false;
  EXPECT_TRUE(check_violation(&rs, labels));
  EXPECT_FALSE(check_final(rs));
}

const std::string formulas[] = {"G((mp_oe & X !mp_oe & (w_o | X w_o) & w_e) -> (!mp_e W mp_o)) & G!((mp_oe & mp_e)|(mp_e & mp_o)|(mp_oe & mp_o)|(w_e & mp_e))"};

INSTANTIATE_TEST_CASE_P(ZipperMergeFormulaTest, ZipperMergeFormula,
    testing::ValuesIn(formulas));

int main(int argc, char **argv) {
  google::AllowCommandLineReparsing();
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}