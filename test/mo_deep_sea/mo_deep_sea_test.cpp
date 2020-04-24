//
// Created by luis on 08.10.19.
//

#define UNIT_TESTING
#define DEBUG
#define PLAN_DEBUG_INFO

#include "gtest/gtest.h"
#include "test/mo_deep_sea/mo_deep_sea_state.hpp"

MctsParameters make_default_mcts_parameters() {
  MctsParameters param;

  param.REWARD_VEC_SIZE = 4;

  param.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC = 1;
  param.random_heuristic.MAX_NUMBER_OF_ITERATIONS = 1000;
  param.COOP_FACTOR = 0.0;
  param.DISCOUNT_FACTOR = 0.9;

  param.uct_statistic.PROGRESSIVE_WIDENING_ENABLED = false;
  param.uct_statistic.PROGRESSIVE_WIDENING_ALPHA = 0.5;

  param.uct_statistic.EXPLORATION_CONSTANT = Eigen::VectorXd::Constant(param.REWARD_VEC_SIZE, 0.7);
  param.uct_statistic.LOWER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);
  param.uct_statistic.UPPER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);

  //  param.e_greedy_uct_statistic_.EPSILON = 0.1;
  //
  //  param.slack_uct_statistic_.ALPHA = 0.05;
  //
  //  param.thres_uct_statistic_.THRESHOLD = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);
  //  param.thres_uct_statistic_.THRESHOLD << -0.44, -0.541, -0.99, 0.0f, std::numeric_limits<ObjectiveVec::Scalar>::max();

  return param;
}

class DeepSeaTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sea_map_.emplace_back(MODSMapElement{1, 1.0f});
    sea_map_.emplace_back(MODSMapElement{2, 2.0f});
    sea_map_.emplace_back(MODSMapElement{3, 3.0f});
    sea_map_.emplace_back(MODSMapElement{4, 5.0f});
    sea_map_.emplace_back(MODSMapElement{4, 8.0f});
    sea_map_.emplace_back(MODSMapElement{4, 16.0f});
    sea_map_.emplace_back(MODSMapElement{7, 24.0f});
    sea_map_.emplace_back(MODSMapElement{7, 50.0f});
    sea_map_.emplace_back(MODSMapElement{8, 74.0f});
    sea_map_.emplace_back(MODSMapElement{10, 124.0f});
    init_pos << 0, 0;
    mcts_parameters_ = make_default_mcts_parameters();
    mcts_parameters_.REWARD_VEC_SIZE = 2;
    mcts_parameters_.uct_statistic.LOWER_BOUND = ObjectiveVec::Zero(2);
    mcts_parameters_.uct_statistic.UPPER_BOUND = ObjectiveVec::Zero(2);
    mcts_parameters_.uct_statistic.LOWER_BOUND << 0.0f, -100.0f;
    mcts_parameters_.uct_statistic.UPPER_BOUND << 124.0f, 0.0f;
  }

  // void TearDown() override {}
  SeaMap sea_map_;
  Eigen::Vector2i init_pos;
  MctsParameters mcts_parameters_;
};

void print_solution(std::vector<Eigen::Vector2i> &pos_history, SeaMap &sea_map) {
  int num_cols = sea_map.size();
  int num_rows = std::max_element(sea_map.begin(), sea_map.end(), [](MODSMapElement a, MODSMapElement b) -> bool {
    return (a.row < b.row);
  })->row + 1;
  std::ostringstream out;
  out << std::fixed;
  out.precision(0);
  for (int row = 0; row < num_rows; row++) {
    out << "|";
    for (int col = 0; col < num_cols; col++) {
      if (std::find(pos_history.begin(), pos_history.end(), Eigen::Vector2i(row, col)) != pos_history.end()) {
        out << " O ";
      } else if (row < sea_map[col].row) {
        out << "   ";
      } else if (row == sea_map[col].row) {
        out << std::setw(3) << sea_map[col].reward;
      } else {
        out << " X ";
      }
      out << "|";
    }
    out << "\n";
    for (int col = 0; col < num_cols; col++) {
      out << "----";
    }
    out << "\n";
  }
  std::cout << out.str() << std::endl;
}

TEST_F(DeepSeaTest, move
) {
  MoDeepSeaState init_state(sea_map_, init_pos);
  std::vector<Reward> rewards(1, Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
  JointAction jt;
  jt.resize(1);
  jt[0] = 3; //Right
  std::shared_ptr<MoDeepSeaState> state = init_state.execute(jt, rewards);
  ASSERT_EQ(state->get_ego_pos(), Eigen::Vector2i(0, 1));

  jt[0] = 0; //Up
  state = state->execute(jt, rewards);
  // Should stay in the same cell
  ASSERT_EQ(state->get_ego_pos(), Eigen::Vector2i(0, 1));

  jt[0] = 1; //Down
  state = state->execute(jt, rewards);
  ASSERT_EQ(state->get_ego_pos(), Eigen::Vector2i(1, 1));

  jt[0] = 3; //Right
  state = state->execute(jt, rewards);
  ASSERT_EQ(state->get_ego_pos(), Eigen::Vector2i(1, 2));

  jt[0] = 2; //Left
  state = state->execute(jt, rewards);
  ASSERT_EQ(state->get_ego_pos(), Eigen::Vector2i(1, 1));

  jt[0] = 1; //Down
  state = state->execute(jt, rewards);
  ASSERT_EQ(state->get_ego_pos(), Eigen::Vector2i(2, 1));
  ASSERT_TRUE(state->is_terminal());
  ASSERT_EQ(rewards[0](0), sea_map_[1].reward);
}

TEST_F(DeepSeaTest, general
) {
Mcts<MoDeepSeaState, UctStatistic<>, UctStatistic<>, RandomHeuristic> mcts(mcts_parameters_);
  auto state = std::make_shared<MoDeepSeaState>(sea_map_, init_pos);
  std::vector<Reward> rewards(1, Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
  JointAction jt;
  jt.resize(1);
  std::vector<Eigen::Vector2i> pos_history;
  pos_history.clear();
  pos_history.emplace_back(state->get_ego_pos());
  while (!state->is_terminal()) {
    mcts.search(*state, 50000, 10000);
jt[0] = mcts.
returnBestAction()[0];
    state = state->execute(jt, rewards);
    pos_history.emplace_back(state->get_ego_pos());
  }
  std::cout << "Positions:" << std::endl;
  for (auto p : pos_history) {
    std::cout << p << ", ";
  }
  std::cout << std::endl;
  std::cout << "END" << std::endl;
  print_solution(pos_history, sea_map_);
  ASSERT_EQ(pos_history.size(), 20);
  ASSERT_EQ((pos_history.end() - 1)->operator()(0), 10);
  ASSERT_EQ((pos_history.end() - 1)->operator()(1), 9);
  //mcts.printTreeToDotFile("test_mo_deep_sea");
}