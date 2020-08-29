//
// Created by Luis Gressenbuch on 18.02.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "evaluation.h"

#include <utility>
#include "glog/logging.h"

namespace mvmcts {
namespace evaluation {

struct ThresholdComparator {
  explicit ThresholdComparator(Eigen::VectorXf thr) : thr_(std::move(thr)) {}
  const Eigen::VectorXf thr_;
  int operator()(Eigen::VectorXf const& a, Eigen::VectorXf const& b) const {
    assert(a.rows() == b.rows() && a.rows() == thr_.rows());
    int i = 0;
    for (auto ai = a.begin(), bi = b.begin(), thri = thr_.begin();
         ai != a.end(); ++ai, ++bi, ++thri, ++i) {
      if ((*ai > *thri && *bi > *thri) || (*ai == *bi)) {
        continue;
      } else {
        return i;
      }
    }
    return i - 1;
  }
};

QValWriter::QValWriter(Eigen::VectorXf thres, const std::string& filename,
                       unsigned long num_actions)
    : thres_(std::move(thres)), timestamp_(0) {
  ofstream_.open(filename);
  ofstream_ << "tstep\t";
  for (unsigned long a = 0; a < num_actions; ++a) {
    for (unsigned long l = 0; l < thres_.size(); ++l) {
      ofstream_ << "a" << a << "_l" << l << "\t";
    }
  }
  ofstream_ << "best_action\tdecision_lvl\n";
  ofstream_.flush();
}

void QValWriter::WriteQVal(
    const std::map<unsigned long, Eigen::VectorXf>& action_val_map,
    unsigned long best_action) {
  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t");
  ofstream_ << timestamp_ << "\t";
  for (auto const& pair : action_val_map) {
    ofstream_ << pair.second.transpose().format(fmt) << "\t";
  }
  ofstream_ << best_action << "\t" << FindLexMax(action_val_map, best_action)
            << "\n";
  ++timestamp_;
}

QValWriter::~QValWriter() { ofstream_.close(); }

int QValWriter::FindLexMax(
    const std::map<unsigned long, Eigen::VectorXf>& action_val_map,
    unsigned long best_action) {
  ThresholdComparator comp(thres_);
  int decision_level = -1;
  for (const auto& item : action_val_map) {
    if (best_action == item.first) {
      continue;
    }
    decision_level = std::max(
        decision_level, comp(action_val_map.at(best_action), item.second));
  }
  return decision_level;
}

}  // namespace evaluation
}  // namespace mvmcts
