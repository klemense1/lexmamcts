// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_STATISTICS_H
#define MCTS_STATISTICS_H

#include "state.h"
#include <map>
#include "common.h"

namespace mcts {



template <class Implementation>
class NodeStatistic
{
public:
    MCTS_TEST;

    NodeStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : num_actions_(num_actions), mcts_parameters_(mcts_parameters) {}
    template <class S>
    ActionIdx ChooseNextAction(const StateInterface<S>& state, std::vector<int>& unexpanded_actions, unsigned int iteration);
    void UpdateStatistic(const NodeStatistic<Implementation>& changed_child_statistic); // update statistic during backpropagation from child node
    void UpdateFromHeuristic(const NodeStatistic<Implementation>& heuristic_statistic); // update statistic during backpropagation from heuristic estimate
    ActionIdx GetBestAction();

    void SetHeuristicEstimate(const Reward& accum_rewards);

    void CollectReward(const Reward& reward, const ActionIdx& action_idx);

    std::map<ActionIdx,Reward> GetExpectedRewards() const;

    std::string PrintNodeInformation() const;
    std::string PrintEdgeInformation(const ActionIdx& action) const;

    Implementation& Impl();
    const Implementation& Impl() const;

public:
    std::pair<ActionIdx, Reward> collected_reward_;
    ActionIdx num_actions_;
    MctsParameters const mcts_parameters_;
};

template <class Implementation>
Implementation& NodeStatistic<Implementation>::Impl() {
    return *static_cast<Implementation*>(this);
}


template <class Implementation>
const Implementation& NodeStatistic<Implementation>::Impl() const {
    return *static_cast<const Implementation*>(this);
}

template <class Implementation>
template <class S>
ActionIdx NodeStatistic<Implementation>::ChooseNextAction(const StateInterface<S>& state, std::vector<int>& unexpanded_actions,
    unsigned int iteration)  {
    return Impl().choose_next_action(state, unexpanded_actions, iteration);
}

template <class Implementation>
ActionIdx NodeStatistic<Implementation>::GetBestAction()  {
    return Impl().get_best_action();
}

template <class Implementation>
std::string NodeStatistic<Implementation>::PrintNodeInformation() const {
    return Impl().print_node_information();
}

template <class Implementation>
std::string NodeStatistic<Implementation>::PrintEdgeInformation(const ActionIdx& action) const {
    return Impl().print_edge_information();
}

template <class Implementation>
void NodeStatistic<Implementation>::UpdateStatistic(const NodeStatistic<Implementation> &changed_child_statistic) {
    return Impl().update_statistic(changed_child_statistic);
}

template <class Implementation>
void NodeStatistic<Implementation>::UpdateFromHeuristic(const NodeStatistic<Implementation>& heuristic_statistic) {
    return Impl().update_from_heuristic(heuristic_statistic);
}

template <class Implementation>
void NodeStatistic<Implementation>::CollectReward(const mcts::Reward &reward, const ActionIdx& action_idx) {
    collected_reward_= std::pair<ActionIdx, Reward>(action_idx, reward);
}

template <class Implementation>
void NodeStatistic<Implementation>::SetHeuristicEstimate(const Reward& accum_rewards) {
    return Impl().set_heuristic_estimate(accum_rewards);
}
template<class Implementation>
std::map<ActionIdx,Reward> NodeStatistic<Implementation>::GetExpectedRewards() const {
  return Impl().get_expected_rewards();
}

} // namespace mcts



#endif