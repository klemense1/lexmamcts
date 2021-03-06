// Copyright (c) 2019 Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef EVALUATION_EVALUATION_H_
#define EVALUATION_EVALUATION_H_

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include "Eigen/Core"

namespace mvmcts {
namespace evaluation {
class QValWriter {
 public:
  QValWriter(Eigen::VectorXd thres,
             const std::string& filename = "/tmp/q_val.dat",
             unsigned long num_actions = 4);
  virtual ~QValWriter();
  void WriteQVal(const std::map<unsigned long, Eigen::VectorXd>& action_val_map,
                 unsigned long best_action);

 private:
  int FindLexMax(const std::map<unsigned long, Eigen::VectorXd>& action_val_map,
                 unsigned long best_action);
  Eigen::VectorXd thres_;
  int timestamp_;
  std::ofstream ofstream_;
  std::string filename_;
};
}  // namespace evaluation
}  // namespace mvmcts

#endif  //  EVALUATION_EVALUATION_H_
