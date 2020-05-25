//
// Created by luis on 08.10.19.
//

#ifndef MAMCTS_TEST_EXTERNAL_MO_DEEP_SEA_STATE_HPP_
#define MAMCTS_TEST_EXTERNAL_MO_DEEP_SEA_STATE_HPP_

#include "Eigen/Core"

#include "mcts/heuristics/random_heuristic.h"
#include "mcts/statistics/uct_statistic.h"

using namespace mcts;

struct MODSMapElement {
  int row;
  float reward;
};

typedef std::vector<MODSMapElement> SeaMap;

class MoDeepSeaState : public mcts::StateInterface<MoDeepSeaState> {
 public:
  MoDeepSeaState(const std::vector<MODSMapElement> &sea_map, const Eigen::Vector2i &ego_pos, int step_counter = 0);
  virtual std::shared_ptr<MoDeepSeaState> Execute(const JointAction &joint_action, std::vector<Reward> &rewards) const;
  virtual std::shared_ptr<MoDeepSeaState> Clone() const;
  virtual ActionIdx GetNumActions(AgentIdx agent_idx) const;
  virtual bool IsTerminal() const;
  virtual const std::vector<AgentIdx> GetAgentIdx() const;
  virtual std::string PrintState() const;
  virtual ~MoDeepSeaState() = default;
  const Eigen::Vector2i &GetEgoPos() const;
  std::vector<Reward> GetFinalReward() const;
 private:
  SeaMap sea_map;
  Eigen::Vector2i ego_pos;
  int step_counter;
  // Up, Down, Left, Right
  const Eigen::Vector2i MODSActions[4] = {Eigen::Vector2i(-1, 0), Eigen::Vector2i(1, 0), Eigen::Vector2i(0, -1), Eigen::Vector2i(0, 1)};
};

#endif //MAMCTS_TEST_EXTERNAL_MO_DEEP_SEA_STATE_HPP_
