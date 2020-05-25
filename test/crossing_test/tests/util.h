//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_TESTS_UTIL_H_
#define MAMCTS_TEST_CROSSING_TEST_TESTS_UTIL_H_

#include <vector>
#include "Eigen/Core"

#include "mcts/mcts.h"

using mcts::MctsParameters;
using mcts::Reward;

Eigen::MatrixXf RewardsToMat(std::vector<Reward> const &rewards);
MctsParameters MakeDefaultMctsParameters();

#endif //MAMCTS_TEST_CROSSING_TEST_TESTS_UTIL_H_
