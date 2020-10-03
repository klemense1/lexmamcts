// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_RANDOM_GENERATOR_H_
#define MVMCTS_RANDOM_GENERATOR_H_

#include <random>

namespace mvmcts {

class RandomGenerator {
 public:
  static std::mt19937 random_generator_;

 public:
  RandomGenerator() {}

  ~RandomGenerator() {}
};
}  // namespace mvmcts

#endif  // MVMCTS_RANDOM_GENERATOR_H_
