// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef RANDOM_HEURISTIC_H
#define RANDOM_HEURISTIC_H

#include "mcts/mcts.h"
#include "mcts/mcts_parameters.h"
#include <iostream>
#include <chrono>

namespace mcts {
// assumes all agents have equal number of actions and the same node statistic
class RandomHeuristic : public mcts::Heuristic<RandomHeuristic>, mcts::RandomGenerator {
public:
  RandomHeuristic(MctsParameters const &mcts_parameters) : Heuristic<RandomHeuristic>(mcts_parameters) {};

  template<class S, class SE, class SO, class H>
  std::vector<SE> get_heuristic_values(const std::shared_ptr<StageNode<S, SE, SO, H>> &node) {
    //catch case where newly expanded state is terminal
    if (node->get_state()->is_terminal()) {
      const AgentIdx num_agents = node->get_state()->get_agent_idx().size();
      std::vector<SE> statistics;
      for (AgentIdx ai = 0; ai < num_agents; ++ai) {
        statistics.emplace_back(node->get_state()->get_num_actions(ai), mcts_parameters_);
        statistics.at(ai).set_heuristic_estimate(Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
      }
      return statistics;
    }

    namespace chr = std::chrono;
    auto start = std::chrono::high_resolution_clock::now();
    std::shared_ptr<S> state = node->get_state()->clone();
    const AgentIdx num_agents = node->get_state()->get_agent_idx().size();

    std::vector<Reward> accum_rewards(num_agents, Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
    std::vector<Reward> step_rewards(num_agents, Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
    const double k_discount_factor = mcts_parameters_.DISCOUNT_FACTOR;
    double modified_discount_factor = k_discount_factor;
    int num_iterations = 0;
    while ((!state->is_terminal()) && (num_iterations < mcts_parameters_.random_heuristic.MAX_NUMBER_OF_ITERATIONS)
           && (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start).count() < mcts_parameters_.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC)) {
      auto new_state = state->execute(random_joint_action(state, num_agents), step_rewards);
      state = new_state->clone();
      num_iterations += 1;
      // discount the rewards of the current step
      for (uint i = 0; i < step_rewards.size(); i++) {
        step_rewards.at(i) = step_rewards.at(i) * modified_discount_factor;
      }
      accum_rewards += step_rewards;
      modified_discount_factor = modified_discount_factor * k_discount_factor;

    }

    accum_rewards += state->get_final_reward();

    Reward coop_sum = Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE);
    coop_sum = std::accumulate(accum_rewards.begin(), accum_rewards.end(), coop_sum);
    coop_sum = coop_sum * mcts_parameters_.COOP_FACTOR;
    // generate an extra node statistic for each agent
    std::vector<SE> statistics;
    for (AgentIdx ai = 0; ai < num_agents; ++ai) {
      statistics.emplace_back(node->get_state()->get_num_actions(ai), mcts_parameters_);
      statistics.at(ai).set_heuristic_estimate((coop_sum + (1.0f - mcts_parameters_.COOP_FACTOR) * accum_rewards.at(ai))
                                                   / (1.0 + mcts_parameters_.COOP_FACTOR * (num_agents - 1)));
    }

    return statistics;
  }

  template<class S>
  JointAction random_joint_action(const std::shared_ptr<S> &state, AgentIdx num_agents) {
    JointAction ja(num_agents);
    for(size_t i = 0; i < num_agents; ++i) {
      std::uniform_int_distribution<ActionIdx> random_action_selection(0,state->get_num_actions(i)-1);
      ja[i] = random_action_selection(random_generator_);
    }
    return ja;
  }

};

} // namespace mcts

#endif