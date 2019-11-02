//
// Created by Luis Gressenbuch on 01.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "common.h"

#include "glog/logging.h"

std::vector<Reward> get_optimal_reward(std::shared_ptr<CrossingState> const init_state) {
  std::shared_ptr<CrossingState> state = init_state->clone();
  JointAction jt(state->get_agent_idx().size(), aconv(Actions::FORWARD));
  std::vector<Reward> rewards(state->get_agent_idx().size(), Reward::Zero());
  std::vector<Reward> accu_reward(state->get_agent_idx().size(), Reward::Zero());

  jt[state->ego_agent_idx] = aconv(Actions::WAIT);
  for (int i = 0; i < 2; ++i) {
    state = state->execute(jt, rewards);
    accu_reward += rewards;
    DLOG(INFO) << state->sprintf();
  }

  jt[state->ego_agent_idx] = aconv(Actions::FORWARD);
  for (; !state->is_terminal(); state = state->execute(jt, rewards)) {
    DLOG(INFO) << state->sprintf();
    accu_reward += rewards;
  }

  accu_reward += state->get_final_reward();
  DLOG(INFO) << "Optimal rewards:";
  DLOG(INFO) << "Ego:" << accu_reward.at(0).transpose();
  for (size_t otr_idx = 1; otr_idx < state->num_other_agents + 1; ++otr_idx) {
    DLOG(INFO) << "Agent " << otr_idx << ":" << accu_reward.at(otr_idx).transpose();
  }
  return accu_reward;
}