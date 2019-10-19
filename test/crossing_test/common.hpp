//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_COMMON_HPP_
#define MAMCTS_TEST_CROSSING_TEST_COMMON_HPP_

#include <map>
#include "mcts/state.h"

using namespace mcts;

enum class Actions {
  WAIT = 0,
  FORWARD = 1,
  //FASTFORWARD = 2,
      BACKWARD = -1,
  NUM = 3
};

const std::map<ActionIdx, Actions> idx_to_action = {
    {0, Actions::WAIT},
    {1, Actions::FORWARD},
    //{2, Actions::FASTFORWARD},
    {2, Actions::BACKWARD}
};

const std::map<Actions, ActionIdx> action_to_idx = {
    {Actions::WAIT, 0},
    {Actions::FORWARD, 1},
    //{Actions::FASTFORWARD, 2},
    {Actions::BACKWARD, 2}
};

static inline Actions aconv(const ActionIdx &action) {
  return idx_to_action.at(action);
}

static inline ActionIdx aconv(const Actions &action) {
  return action_to_idx.at(action);
}

typedef struct AgentState {
  AgentState() : x_pos(0), last_action(Actions::WAIT) {}
  AgentState(const int &x, const Actions &last_action) :
      x_pos(x), last_action(last_action) {}
  int x_pos;
  Actions last_action;
} AgentState;

typedef std::pair<AgentState, std::vector<AgentState>> World;

#endif //MAMCTS_TEST_CROSSING_TEST_COMMON_HPP_
