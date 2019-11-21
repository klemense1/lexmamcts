//
// Created by Luis Gressenbuch on 09.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_MCTS_HEURISTICS_SEMI_RANDOM_HEURISTIC_H_
#define MAMCTS_MCTS_HEURISTICS_SEMI_RANDOM_HEURISTIC_H_

#include "mcts/mcts.h"
#include "mcts/mcts_parameters.h"
#include <iostream>
#include <chrono>

namespace mcts {
// assumes all agents have equal number of actions and the same node statistic
class SemiRandomHeuristic : public mcts::Heuristic<SemiRandomHeuristic>, mcts::RandomGenerator {
 public:
  SemiRandomHeuristic(MctsParameters const &mcts_parameters) : Heuristic<SemiRandomHeuristic>(mcts_parameters) {};

  template<class S, class SE, class SO, class H>
  std::vector<SE> get_heuristic_values(const std::shared_ptr<StageNode<S, SE, SO, H>> &node) {
    //catch case where newly expanded state is terminal
    if (node->get_state()->is_terminal()) {
      const AgentIdx num_agents = node->get_state()->get_agent_idx().size();
      const ActionIdx num_actions = node->get_state()->get_num_actions(S::ego_agent_idx);
      std::vector<SE> statistics(num_agents, SE(num_actions, mcts_parameters_));
      for (AgentIdx ai = 0; ai < num_agents; ++ai) {
        statistics.at(ai).set_heuristic_estimate(Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
      }
      return statistics;
    }

    namespace chr = std::chrono;
    auto start = std::chrono::high_resolution_clock::now();
    std::shared_ptr<S> state = node->get_state()->clone();
    const AgentIdx num_agents = node->get_state()->get_agent_idx().size();
    const ActionIdx num_actions = node->get_state()->get_num_actions(S::ego_agent_idx);

    std::vector<Reward> accum_rewards(num_agents, Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
    std::vector<Reward> step_rewards(num_agents, Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE));
    const double k_discount_factor = mcts_parameters_.DISCOUNT_FACTOR;
    double modified_discount_factor = k_discount_factor;
    int num_iterations = 0;
    while ((!state->is_terminal()) && (num_iterations < mcts_parameters_.random_heuristic.MAX_NUMBER_OF_ITERATIONS)
        && (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start).count()
            < mcts_parameters_.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC)) {
      auto state_reward_pair = apply_best_ego_action(state, random_joint_action(num_actions, num_agents));
      state = state_reward_pair.state;
      num_iterations += 1;
      // discount the rewards of the current step
      for (uint i = 0; i < step_rewards.size(); i++) {
        state_reward_pair.reward.at(i) = state_reward_pair.reward.at(i) * modified_discount_factor;
      }
      accum_rewards += state_reward_pair.reward;
      modified_discount_factor = modified_discount_factor * k_discount_factor;
    }

    accum_rewards += state->get_final_reward();

    Reward coop_sum = Reward::Zero(mcts_parameters_.REWARD_VEC_SIZE);
    coop_sum = std::accumulate(accum_rewards.begin(), accum_rewards.end(), coop_sum);
    coop_sum = coop_sum * mcts_parameters_.COOP_FACTOR;
    // generate an extra node statistic for each agent
    std::vector<SE> statistics(num_agents, SE(num_actions, mcts_parameters_));
    for (AgentIdx ai = 0; ai < num_agents; ++ai) {
      statistics.at(ai).set_heuristic_estimate(coop_sum + (1.0f - mcts_parameters_.COOP_FACTOR) * accum_rewards.at(ai));
    }

    return statistics;
  }

  JointAction random_joint_action(ActionIdx num_actions, AgentIdx num_agents) {
    std::uniform_int_distribution<ActionIdx> random_action_selection(0, num_actions - 1);

    auto gen = [&]() {
      return random_action_selection(random_generator_);
    };

    JointAction ja(num_agents);
    std::generate(std::begin(ja), std::end(ja), gen);
    return ja;
  }

 private:

  template<class S>
  struct StateRewardPair {
    std::shared_ptr<S> state;
    JointReward reward;
  };

  template<class S>
  StateRewardPair<S> apply_best_ego_action(std::shared_ptr<S> const state, JointAction jt) {
    ActionIdx num_ego_actions = state->get_num_actions(S::ego_agent_idx);
    std::vector<StateRewardPair<S>> state_reward_vec(num_ego_actions);
    std::shared_ptr<S> next_state;
    JointReward next_reward;
    for (ActionIdx a = 0; a < num_ego_actions; ++a) {
      jt[S::ego_agent_idx] = a;
      next_state = state->execute(jt, next_reward);
      state_reward_vec[a] = {next_state, next_reward};
    }
    auto max_element = std::max_element(state_reward_vec.begin(),
                                        state_reward_vec.end(),
                                        [](StateRewardPair<S> const &a, StateRewardPair<S> const &b) -> bool {
                                          return (a.reward[S::ego_agent_idx](0) < b.reward[S::ego_agent_idx](0));
                                        });

    return *max_element;
  }

};

} // namespace mcts

#endif //MAMCTS_MCTS_HEURISTICS_SEMI_RANDOM_HEURISTIC_H_
