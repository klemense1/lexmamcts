// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_INTERMEDIATE_NODE_H
#define MCTS_INTERMEDIATE_NODE_H

#include "state.h"
#include "node_statistic.h"
#include "random_generator.h"
#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>

namespace mcts {

/*
 * @tparam S State Model
 * @tparam Stats Statistics Model responsible for selection, expansion and update
 */
template<class S, class Stats>
class IntermediateNode : public Stats {
 private:

  std::vector<int> unexpanded_actions_; // contains all action indexes which have not been expanded yet
  AgentIdx agent_idx_;
  const StateInterface<S> &state_;

 public:
  IntermediateNode(const StateInterface<S> &state,
                   AgentIdx agent_idx,
                   ActionIdx num_actions,
                   MctsParameters mcts_parameters);

  ~IntermediateNode();

  ActionIdx choose_next_action();

  bool all_actions_expanded();

  AgentIdx get_agent_idx() const;

        MCTS_TEST

};

template<class S, class Stats>
using IntermediateNodePtr = std::shared_ptr<IntermediateNode<S, Stats>>;

template<class S, class Stats>
IntermediateNode<S, Stats>::IntermediateNode(const StateInterface<S> &state,
                                             AgentIdx agent_idx,
                                             ActionIdx num_actions,
                                             MctsParameters mcts_parameters) :
    Stats(num_actions, mcts_parameters),
    unexpanded_actions_(num_actions),
    agent_idx_(agent_idx),
    state_(state) {
  // initialize action indexes from 0 to (number of actions -1)
  std::iota(unexpanded_actions_.begin(), unexpanded_actions_.end(), 0);
}

template<class S, class Stats>
IntermediateNode<S, Stats>::~IntermediateNode() {}

template<class S, class Stats>
ActionIdx IntermediateNode<S, Stats>::choose_next_action() {
        return Stats::choose_next_action(state_, unexpanded_actions_);
}

template<class S, class Stats>
bool IntermediateNode<S, Stats>::all_actions_expanded() {
  return unexpanded_actions_.empty();
}

template<class S, class Stats>
inline AgentIdx IntermediateNode<S, Stats>::get_agent_idx() const { return agent_idx_;}

} // namespace mcts

#endif