//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_COMMON_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_COMMON_H_

#include <vector>
#include <memory>

#include "Eigen/Core"

#include "test/crossing_test/crossing_state.hpp"
#include "mcts/mcts.h"

std::vector<Reward> get_optimal_reward(std::shared_ptr<CrossingState> const state);
Eigen::MatrixXf rewards_to_mat(std::vector<Reward> const &rewards);
MctsParameters make_default_mcts_parameters();

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_COMMON_H_
