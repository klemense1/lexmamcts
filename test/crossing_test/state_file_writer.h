//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_STATE_FILE_WRITER_H_
#define TEST_CROSSING_TEST_STATE_FILE_WRITER_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Core"

class StateFileWriter {
 public:
  StateFileWriter(size_t num_agents, const std::string& filename);
  virtual ~StateFileWriter();
  void write_multi_timestep(const std::vector<Eigen::MatrixXi>& states);

 private:
  int timestamp_;
  std::ofstream ofstream_;
  size_t num_agents_;
  std::string filename_;
};

#endif  // TEST_CROSSING_TEST_STATE_FILE_WRITER_H_
