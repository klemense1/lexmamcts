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
  FASTFORWARD = 2,
  BACKWARD = -1,
  NUM = 4
};

enum Rule {
  NO_COLLISION = 0, REACH_GOAL, NO_SPEEDING, GIVE_WAY, LEAVE_INTERSECTION, REACH_GOAL_FIRST, ZIP, NUM,
};

const std::map<ActionIdx, Actions> idx_to_action = {
    {0, Actions::FORWARD},
    {1, Actions::WAIT},
    {2, Actions::FASTFORWARD},
    {3, Actions::BACKWARD},
};

const std::map<Actions, ActionIdx> action_to_idx = {
    {Actions::FORWARD, 0},
    {Actions::WAIT, 1},
    {Actions::FASTFORWARD, 2},
    {Actions::BACKWARD, 3},
};

static inline Actions aconv(const ActionIdx &action) {
  return idx_to_action.at(action);
}

static inline ActionIdx aconv(const Actions &action) {
  return action_to_idx.at(action);
}

typedef struct AgentState {
  AgentState() : x_pos(0), last_action(Actions::WAIT), lane(lane_counter++) {}
  AgentState(const int &x, const Actions &last_action, int lane) :
      x_pos(x), last_action(last_action), lane(lane) {}
  int x_pos;
  Actions last_action;
  int lane;
  static int lane_counter;
} AgentState;

typedef std::pair<AgentState, std::vector<AgentState>> World;

#endif //MAMCTS_TEST_CROSSING_TEST_COMMON_HPP_
