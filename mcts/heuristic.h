// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_HEURISTIC_H_
#define MCTS_HEURISTIC_H_

#include <memory>
#include <string>
#include <vector>
#include "mcts/node_statistic.h"
#include "mcts/state.h"

namespace mcts {

template <class S, class SE, class SO, class H>
class StageNode;

template <class Implementation>
class Heuristic {
 public:
  explicit Heuristic(MctsParameters const &mcts_parameters)
      : mcts_parameters_(mcts_parameters) {}

  template <class S, class SE, class SO, class H>
  std::vector<SE> GetHeuristicValues(
      const std::shared_ptr<StageNode<S, SE, SO, H>> &node);

  std::string PrintState() const;

 protected:
  MctsParameters mcts_parameters_;

 private:
  Implementation &Impl();

  Implementation &Impl() const;
};

template <class Implementation>
Implementation &Heuristic<Implementation>::Impl() {
  return *static_cast<Implementation *>(this);
}

template <class Implementation>
Implementation &Heuristic<Implementation>::Impl() const {
  return *static_cast<const Implementation *>(this);
}

template <class Implementation>
template <class S, class SE, class SO, class H>
std::vector<SE> Heuristic<Implementation>::GetHeuristicValues(
    const std::shared_ptr<StageNode<S, SE, SO, H>> &node) {
  return Impl().get_heuristic_values(node);
}

template <class Implementation>
std::string Heuristic<Implementation>::PrintState() const {
  return Impl().sprintf();
}

}  // namespace mcts

#endif  // MCTS_HEURISTIC_H_
