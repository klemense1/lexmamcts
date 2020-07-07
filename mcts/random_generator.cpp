// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "mcts/random_generator.h"

std::mt19937 mcts::RandomGenerator::random_generator_ = std::mt19937(1000);
