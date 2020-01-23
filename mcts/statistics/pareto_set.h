//
// Created by Luis Gressenbuch on 30.10.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MCTS_STATISTICS_PARETO_SET_H_
#define MCTS_STATISTICS_PARETO_SET_H_

#include <map>
#include <random>
#include <vector>
#include "mcts/random_generator.h"

namespace mcts {

template <class Key, class T>
class ParetoSet : RandomGenerator {
 public:
  ParetoSet();
  bool add(Key const &index, T const &element);
  void add(std::vector<T> const &elements);
  size_t size() const;
  Key get_random();

 private:
  std::map<Key, T> map_;
  static bool dominates(T const &a, T const &b);
};

template <class Key, class T>
ParetoSet<Key, T>::ParetoSet() {}
template <class Key, class T>
bool ParetoSet<Key, T>::add(Key const &index, T const &element) {
  for (auto it = map_.cbegin(); it != map_.cend();) {
    if (dominates(it->second, element)) {
      // Is dominated by already contained element => not optimal
      return false;
    }
    if (dominates(element, it->second)) {
      // Dominates already contained element => remove dominated element and
      // continue
      // https://stackoverflow.com/questions/8234779/how-to-remove-from-a-map-while-iterating-it
      it = map_.erase(it);
    } else {
      ++it;
    }
  }
  map_[index] = element;
  return true;
}
template <class Key, class T>
Key ParetoSet<Key, T>::get_random() {
  assert(map_.size() > 0);
  std::uniform_int_distribution<size_t> dist(0, map_.size() - 1);
  size_t rnd_index = dist(random_generator_);
  auto it = map_.cbegin();
  // Can't use std::advance for some reason
  for (size_t i = 0; i < rnd_index; ++i) {
    ++it;
  }
  return it->first;
}
template <class Key, class T>
bool ParetoSet<Key, T>::dominates(const T &a, const T &b) {
  bool weak = true;
  for (auto it_a = a.cbegin(), it_b = b.cbegin(); it_a != a.cend();
       ++it_a, ++it_b) {
    if (*it_a < *it_b) {
      return false;
    }
    if (*it_a > *it_b) {
      weak = false;
    }
  }
  return !weak;
}
template <class Key, class T>
size_t ParetoSet<Key, T>::size() const {
  return map_.size();
}
template <class Key, class T>
void ParetoSet<Key, T>::add(const std::vector<T> &elements) {
  for (size_t i = 0; i < elements.size(); i++) {
    add(i, elements.at(i));
  }
}
}  // namespace mcts

#endif  // MCTS_STATISTICS_PARETO_SET_H_
