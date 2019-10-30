#include "gtest/gtest.h"
#include "Eigen/Core"

#include "mcts/statistics/pareto_set.h"

using namespace mcts;
using namespace Eigen;

TEST(ParetoTest, add) {
  ParetoSet<int, Vector3i> pareto_set;
  Vector3i a, b, c, d;
  a << 1, 0, 0;
  b << 0, 0, 1;// Incomparable to a
  c << 1, 1, 0;// Dominates a, incomparable to b
  d << 0, 0, 0;// Dominated by b and c
  ASSERT_TRUE(pareto_set.add(1, a));

  ASSERT_TRUE(pareto_set.add(2, b));
  ASSERT_EQ(pareto_set.size(), 2u);

  ASSERT_TRUE(pareto_set.add(3, c));
  ASSERT_EQ(pareto_set.size(), 2u);

  ASSERT_FALSE(pareto_set.add(4, d));
  ASSERT_EQ(pareto_set.size(), 2u);
}


