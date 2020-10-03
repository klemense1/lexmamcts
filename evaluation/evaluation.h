//
// Created by Luis Gressenbuch on 18.02.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

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
  QValWriter(Eigen::VectorXf thres,
             const std::string& filename = "/tmp/q_val.dat",
             unsigned long num_actions = 4);
  virtual ~QValWriter();
  void WriteQVal(const std::map<unsigned long, Eigen::VectorXf>& action_val_map,
                 unsigned long best_action);

 private:
  int FindLexMax(const std::map<unsigned long, Eigen::VectorXf>& action_val_map,
                 unsigned long best_action);
  Eigen::VectorXf thres_;
  int timestamp_;
  std::ofstream ofstream_;
  std::string filename_;
};
}  // namespace evaluation
}  // namespace mvmcts

#endif  //  EVALUATION_EVALUATION_H_
