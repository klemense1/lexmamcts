//
// Created by luis on 26.10.19.
//

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO

#include "gtest/gtest.h"
#include "mcts/statistics/uct_statistic.h"

using namespace mcts;

TEST(ComperatorTest, slack_comperator) {
  ObjectiveVec a = ObjectiveVec::Ones();
  ObjectiveVec b = ObjectiveVec::Ones();
  ObjectiveVec slack_a = ObjectiveVec::Zero();
  ObjectiveVec slack_b = ObjectiveVec::Zero();
  a *= 10;
  b *= 20;
  slack_a << 5, 5, 0, 0;
  slack_b << 2, 2, 0, 0;
  // No overlap
  ASSERT_TRUE(slack_compare(a, b, slack_a, slack_b));
  slack_b << 5, 5, 0, 0;
  // Overlap
  ASSERT_TRUE(slack_compare(a, b, slack_a, slack_b));
  a(2) = 50;
  ASSERT_FALSE(slack_compare(a, b, slack_a, slack_b));
}

TEST(ComperatorTest, threshold_comperator_random) {
  ObjectiveVec a = ObjectiveVec::Ones();
  ObjectiveVec b = ObjectiveVec::Ones();
  ObjectiveVec thr = ObjectiveVec::Ones() * FLT_MAX;
  std::mt19937 prng(1000);
  std::uniform_int_distribution<int> distr(-100, 100);
  auto gen = [&distr, &prng]() {
    return distr(prng);
  };
  for (size_t i = 0; i < 100; i++) {
    std::generate(a.begin(), a.end(), gen);
    std::generate(b.begin(), b.end(), gen);
    a(0) = b(0);
    ASSERT_EQ(std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()), threshold_compare(a, b, thr));
  }
}

TEST(ComperatorTest, threshold_comperator) {
  ObjectiveVec a;
  ObjectiveVec b;
  ObjectiveVec thr;
  a << 20, 10, 20, 10;
  b << 10, 20, 10, 20;
  thr << 10, 100, 100, 100;
  ASSERT_FALSE(std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()));
  ASSERT_TRUE(threshold_compare(a, b, thr));
}