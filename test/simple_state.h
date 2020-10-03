// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef SIMPLESTATE_H
#define SIMPLESTATE_H

#include <iostream>

using namespace mvmcts;

// A simple environment with a 1D state, only if both agents select different
// actions, they get nearer to the terminal state
class SimpleState : public mvmcts::StateInterface<SimpleState> {
 public:
  SimpleState(int length)
      : state_length_(length),
        winning_state_length_(10),
        loosing_state_length_(-1){};
  ~SimpleState(){};

  std::shared_ptr<SimpleState> Clone() const {
    return std::make_shared<SimpleState>(*this);
  }

  std::shared_ptr<SimpleState> Execute(const JointAction& joint_action,
                                       std::vector<Reward>& rewards) const {
    // normally we map each single action value in joint action with a map to
    // the floating point action. Here, not required
    rewards.resize(2);
    rewards[0](0) = 0;
    rewards[1](0) = 0;
    if (joint_action == JointAction{0, 0} ||
        joint_action == JointAction{1, 1}) {
      return std::make_shared<SimpleState>(*this);
    } else if (joint_action == JointAction{0, 1} ||
               joint_action == JointAction{1, 0}) {
      // rewards[0] = -1.0f; rewards[1] = -1.0f;
      auto new_state_length = state_length_ + 1;

      rewards = std::vector<Reward>(2, Reward::Constant(1, 1));

      if (new_state_length >= winning_state_length_) {
        rewards[0] << 5;
        rewards[1] << 10;
        new_state_length = winning_state_length_;
      }
      return std::make_shared<SimpleState>(new_state_length);
    } else {
      std::cout << "unvalid action selected" << std::endl;
      return std::make_shared<SimpleState>(*this);
    }
  }

  ActionIdx GetNumActions(AgentIdx agent_idx) const { return 2; }

  bool IsTerminal() const {
    return state_length_ >= winning_state_length_ ||
           state_length_ <= loosing_state_length_;
  }

  const std::vector<AgentIdx> GetAgentIdx() const {
    return std::vector<AgentIdx>{0, 1};
  }

  std::string PrintState() const {
    std::stringstream ss;
    ss << "SimpleState (state_length: " << state_length_ << ")";
    return ss.str();
  }

 private:
  int state_length_;

  // PARAMS
  int winning_state_length_;
  int loosing_state_length_;
};

#endif
