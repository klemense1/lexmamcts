// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_INTERMEDIATE_NODE_H_
#define MVMCTS_INTERMEDIATE_NODE_H_

#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>
#include "mvmcts/node_statistic.h"
#include "mvmcts/random_generator.h"
#include "mvmcts/state.h"

namespace mvmcts {

/*
 * @tparam S State Model
 * @tparam Stats Statistics Model responsible for selection, expansion and
 * update
 */
template <class S, class Stats>
class IntermediateNode : public Stats {
 private:
  std::vector<int> unexpanded_actions_;  // contains all action indexes which
                                         // have not been expanded yet
  AgentIdx agent_idx_;
  const StateInterface<S> &state_;

 public:
  IntermediateNode(const StateInterface<S> &state, AgentIdx agent_idx,
                   ActionIdx num_actions,
                   const MvmctsParameters mvmcts_parameters);

  ~IntermediateNode();

  ActionIdx ChooseNextAction(unsigned int iteration);

  bool AllActionsExpanded();

  AgentIdx GetAgentIdx() const;

  MVMCTS_TEST
};

template <class S, class Stats>
using IntermediateNodePtr = std::shared_ptr<IntermediateNode<S, Stats>>;

template <class S, class Stats>
IntermediateNode<S, Stats>::IntermediateNode(
    const StateInterface<S> &state, AgentIdx agent_idx, ActionIdx num_actions,
    const MvmctsParameters mvmcts_parameters)
    : Stats(num_actions,mvmcts_parameters),
      unexpanded_actions_(num_actions),
      agent_idx_(agent_idx),
      state_(state) {
  // initialize action indexes from 0 to (number of actions -1)
  std::iota(unexpanded_actions_.begin(), unexpanded_actions_.end(), 0);
}

template <class S, class Stats>
IntermediateNode<S, Stats>::~IntermediateNode() = default;

template <class S, class Stats>
ActionIdx IntermediateNode<S, Stats>::ChooseNextAction(unsigned int iteration) {
  return Stats::ChooseNextAction(state_, unexpanded_actions_, iteration);
}

template <class S, class Stats>
bool IntermediateNode<S, Stats>::AllActionsExpanded() {
  return unexpanded_actions_.empty();
}

template <class S, class Stats>
inline AgentIdx IntermediateNode<S, Stats>::GetAgentIdx() const {
  return agent_idx_;
}

}  // namespace mvmcts

#endif  // MVMCTS_INTERMEDIATE_NODE_H_
