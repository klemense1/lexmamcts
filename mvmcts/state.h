// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_STATE_H_
#define MVMCTS_STATE_H_

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "mvmcts/common.h"
#include "mvmcts/mvmcts_parameters.h"

namespace mvmcts {

typedef std::size_t ActionIdx;
typedef unsigned char AgentIdx;
typedef std::vector<ActionIdx> JointAction;

typedef Eigen::VectorXf Reward;
typedef std::vector<Reward> JointReward;

template <typename T>
inline std::vector<T> operator+(const std::vector<T>& a,
                                const std::vector<T>& b) {
  MVMCTS_EXPECT_TRUE(a.size() == b.size());
  std::vector<T> result;
  result.reserve(a.size());

  std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(result),
                 std::plus<T>());
  return result;
}

template <typename T>
inline std::vector<T>& operator+=(std::vector<T>& a, const std::vector<T>& b) {
  assert(a.size() == b.size());
  for (uint i = 0; i < b.size(); i++) {
    a[i] = a[i] + b[i];
  }
  return a;
}

inline std::ostream& operator<<(std::ostream& os, const JointAction& a) {
  os << "[";
  for (auto it = a.begin(); it != a.end(); ++it)
    os << static_cast<int>(*it) << " ";
  os << "]";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, JointReward const& d) {
  os << "[";
  for (auto ii = d.begin(); ii != d.end(); ++ii) {
    os << ii->transpose();
    if (ii >= d.end() - 1) {
      break;
    }
    os << ", ";
  }
  os << "]";
  return os;
}

template <typename Implementation>
class StateInterface {
 public:
  std::shared_ptr<Implementation> Execute(const JointAction& joint_action,
                                          std::vector<Reward>& rewards) const;

  std::vector<Reward> GetTerminalReward() const;

  std::shared_ptr<Implementation> Clone() const;

  ActionIdx GetNumActions(AgentIdx agent_idx) const;

  bool IsTerminal() const;

  const std::vector<AgentIdx> GetAgentIdx() const;

  static const AgentIdx ego_agent_idx;

  std::string PrintState() const;

  virtual ~StateInterface() = default;

 private:
  CRTP_INTERFACE(Implementation)
  CRTP_CONST_INTERFACE(Implementation)
};

template <typename Implementation>
inline std::shared_ptr<Implementation> StateInterface<Implementation>::Execute(
    const JointAction& joint_action, std::vector<Reward>& rewards) const {
  return impl().Execute(joint_action, rewards);
}

template <typename Implementation>
inline std::shared_ptr<Implementation> StateInterface<Implementation>::Clone()
    const {
  return impl().Clone();
}

template <typename Implementation>
inline ActionIdx StateInterface<Implementation>::GetNumActions(
    AgentIdx agent_idx) const {
  return impl().GetNumActions(agent_idx);
}

template <typename Implementation>
inline bool StateInterface<Implementation>::IsTerminal() const {
  return impl().IsTerminal();
}

template <typename Implementation>
inline const std::vector<AgentIdx> StateInterface<Implementation>::GetAgentIdx()
    const {
  return impl().GetAgentIdx();
}

template <typename Implementation>
inline std::string StateInterface<Implementation>::PrintState() const {
  return impl().PrintState();
}

template <typename Implementation>
inline std::vector<Reward> StateInterface<Implementation>::GetTerminalReward()
    const {
  return impl().GetTerminalReward();
}

template <typename Implementation>
const AgentIdx StateInterface<Implementation>::ego_agent_idx = 0;

}  // namespace mvmcts

#endif  // MVMCTS_STATE_H_
