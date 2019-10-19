//
// Created by luis on 19.10.19.
//

#include "random_generator.h"

using namespace mcts;

std::mt19937 RandomGenerator::random_generator_ = std::mt19937(1000);