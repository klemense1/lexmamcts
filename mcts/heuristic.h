// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_HEURISTIC_H
#define MCTS_HEURISTIC_H


#include <memory>
#include "state.h"
#include "node_statistic.h"

namespace mcts {

template<class S, class SE, class SO, class H>
class StageNode;


template<class Implementation>
class Heuristic {
public:
  Heuristic(MctsParameters const &mcts_parameters) : mcts_parameters_(mcts_parameters) {};

  template<class S, class SE, class SO, class H>
  std::vector<SE> get_heuristic_values(const std::shared_ptr<StageNode<S, SE, SO, H>> &node);

  std::string sprintf() const;

protected:
  MctsParameters mcts_parameters_;

private:

  Implementation &impl();

  Implementation &impl() const;
};


template<class Implementation>
Implementation &Heuristic<Implementation>::impl() {
  return *static_cast<Implementation *>(this);
}


template<class Implementation>
Implementation &Heuristic<Implementation>::impl() const {
  return *static_cast<const Implementation *>(this);
}

template<class Implementation>
template<class S, class SE, class SO, class H>
std::vector<SE> Heuristic<Implementation>::get_heuristic_values(const std::shared_ptr<StageNode<S, SE, SO, H>> &node) {
  return impl().get_heuristic_values(node);
}

template<class Implementation>
std::string Heuristic<Implementation>::sprintf() const {
  return impl().sprintf();
}

} // namespace mcts



#endif