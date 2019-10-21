// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "mcts/mcts_parameters.h"

namespace mcts {

double MctsParameters::DISCOUNT_FACTOR = 0.9;
double MctsParameters::EXPLORATION_CONSTANT = 0.7;

double MctsParameters::MAX_SEARCH_TIME_RANDOM_HEURISTIC = 50000;
double MctsParameters::MAX_NUMBER_OF_ITERATIONS_RANDOM_HEURISTIC = 10000;

ObjectiveVec MctsParameters::LOWER_BOUND = Eigen::Vector4f(-1000.0f, -1000.0f, 0.0f, -1000.0f);
ObjectiveVec MctsParameters::UPPER_BOUND = Eigen::Vector4f(0.0f, 0.0f, 100.0f, 0.0f);

double MctsParameters::COOP_FACTOR = 0.0f;

} // namespace mcts
