//
// Created by luis on 08.10.19.
//

#include "test/mo_deep_sea/mo_deep_sea_state.hpp"
std::shared_ptr<MoDeepSeaState> MoDeepSeaState::execute(const JointAction &joint_action,
                                                        std::vector<Reward> &rewards) const {
  assert(!is_terminal());
  int num_cols = sea_map.size() - 1;
  Eigen::Vector2i tmp_pos = ego_pos + MODSActions[joint_action[0]];
  Eigen::Vector2i new_pos;
  int clamped_col = std::min(std::max(tmp_pos(1), 0), num_cols);
  new_pos << std::min(std::max(tmp_pos(0), 0), sea_map[clamped_col].row),
      clamped_col;
  std::shared_ptr<MoDeepSeaState> new_state = std::make_shared<MoDeepSeaState>(sea_map, new_pos, step_counter + 1);
  rewards.resize(2);
  if (new_state->is_terminal()) {
    rewards[0] << sea_map[new_pos(1)].reward, -step_counter;
  } else {
    rewards[0] << 0.0f, 0.0f;
  }
  return new_state;
}
std::shared_ptr<MoDeepSeaState> MoDeepSeaState::clone() const {
  return std::make_shared<MoDeepSeaState>(*this);
}
ActionIdx MoDeepSeaState::get_num_actions(AgentIdx agent_idx) const {
  // Up, Down, Left, Right
  return 4;
}
bool MoDeepSeaState::is_terminal() const {
  return (ego_pos(0) >= sea_map[ego_pos(1)].row || step_counter >= 100);
}
const std::vector<AgentIdx> MoDeepSeaState::get_agent_idx() const {
  return std::vector<AgentIdx>{0};
}
std::string MoDeepSeaState::sprintf() const {
  int num_cols = sea_map.size();
  int num_rows = std::max_element(sea_map.begin(), sea_map.end(), [](MODSMapElement a, MODSMapElement b) -> bool {
    return (a.row < b.row);
  })->row + 1;
  std::ostringstream out;
  out << std::fixed;
  out.precision(0);
  out << "\n";
  for (int row = 0; row < num_rows; row++) {
    out << "|";
    for (int col = 0; col < num_cols; col++) {
      if (row < sea_map[col].row) {
        out << "   ";
      } else if (row == sea_map[col].row) {
        out << std::setw(3) << sea_map[col].reward;
      } else {
        out << " X ";
      }
      out << "|";
    }
    out << "\n";
    for (int col = 0; col < num_cols; col++) {
      out << "----";
    }
    out << "\n";
  }
  out << "\n";
  return out.str();
}
MoDeepSeaState::~MoDeepSeaState() {

}
MoDeepSeaState::MoDeepSeaState(const std::vector<MODSMapElement> &sea_map,
                               const Eigen::Vector2i &ego_pos,
                               int step_counter) : sea_map(sea_map), ego_pos(ego_pos), step_counter(step_counter) {}
const Eigen::Vector2i &MoDeepSeaState::get_ego_pos() const {
  return ego_pos;
}
