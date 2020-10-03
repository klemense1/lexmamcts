// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_HEURISTICS_RANDOM_HEURISTIC_H_
#define MVMCTS_HEURISTICS_RANDOM_HEURISTIC_H_

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "mvmcts/mvmcts.h"
#include "mvmcts/mvmcts_parameters.h"

namespace mvmcts {
// assumes all agents have equal number of actions and the same node statistic
class RandomHeuristic : public mvmcts::Heuristic<RandomHeuristic>,
                        mvmcts::RandomGenerator {
 public:
  explicit RandomHeuristic(MvmctsParameters const mvmcts_parameters)
      : Heuristic<RandomHeuristic>(mvmcts_parameters) {}

  template <class S, class SE, class SO, class H>
  std::vector<SE> GetHeuristicValues(
      const std::shared_ptr<StageNode<S, SE, SO, H>> &node) {
    // catch case where newly expanded state is terminal
    if (node->GetState()->IsTerminal()) {
      const AgentIdx num_agents = node->GetState()->GetAgentIdx().size();
      std::vector<SE> statistics;
      for (AgentIdx ai = 0; ai < num_agents; ++ai) {
        statistics.emplace_back(node->GetState()->GetNumActions(ai),
                               mvmcts_parameters_);
        statistics.at(ai).SetHeuristicEstimate(
            Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
      }
      return statistics;
    }

    namespace chr = std::chrono;
    auto start = std::chrono::high_resolution_clock::now();
    std::shared_ptr<S> state = node->GetState()->Clone();
    const AgentIdx num_agents = node->GetState()->GetAgentIdx().size();

    std::vector<Reward> accum_rewards(
        num_agents, Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
    std::vector<Reward> step_rewards(
        num_agents, Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE));
    const double k_discount_factor =mvmcts_parameters_.DISCOUNT_FACTOR;
    double modified_discount_factor = k_discount_factor;
    int num_iterations = 0;
    while (
        (!state->IsTerminal()) &&
        (num_iterations <
        mvmcts_parameters_.random_heuristic.MAX_NUMBER_OF_ITERATIONS) &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now() - start)
             .count() <
        mvmcts_parameters_.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC)) {
      auto new_state =
          state->Execute(RandomJointAction(state, num_agents), step_rewards);
      state = new_state->Clone();
      num_iterations += 1;
      // discount the rewards of the current step
      for (uint i = 0; i < step_rewards.size(); i++) {
        step_rewards.at(i) = step_rewards.at(i) * modified_discount_factor;
      }
      accum_rewards += step_rewards;
      modified_discount_factor = modified_discount_factor * k_discount_factor;
    }

    accum_rewards += state->GetTerminalReward();

    Reward coop_sum = Reward::Zero(mvmcts_parameters_.REWARD_VEC_SIZE);
    coop_sum =
        std::accumulate(accum_rewards.begin(), accum_rewards.end(), coop_sum);
    coop_sum = coop_sum *mvmcts_parameters_.COOP_FACTOR;
    // generate an extra node statistic for each agent
    std::vector<SE> statistics;
    for (AgentIdx ai = 0; ai < num_agents; ++ai) {
      statistics.emplace_back(node->GetState()->GetNumActions(ai),
                             mvmcts_parameters_);
      statistics.at(ai).SetHeuristicEstimate(
          (coop_sum +
           (1.0f -mvmcts_parameters_.COOP_FACTOR) * accum_rewards.at(ai)));
    }

    return statistics;
  }

  template <class S>
  JointAction RandomJointAction(const std::shared_ptr<S> &state,
                                AgentIdx num_agents) {
    JointAction ja(num_agents);
    for (size_t i = 0; i < num_agents; ++i) {
      std::uniform_int_distribution<ActionIdx> random_action_selection(
          0, state->GetNumActions(i) - 1);
      ja[i] = random_action_selection(random_generator_);
    }
    return ja;
  }
};

}  // namespace mvmcts

#endif  // MVMCTS_HEURISTICS_RANDOM_HEURISTIC_H_
