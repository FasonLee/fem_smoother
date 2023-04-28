#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include "include/discrete_points_reference_line_smoother.h"
#include "include/reference_point.h"
#include "matplotlibcpp.h"

using namespace apollo;
using namespace planning;

void read_data(const std::string &file_name, std::vector<double> &xs, std::vector<double> &ys) {
    std::ifstream file(file_name);
    std::string line;
    std::getline(file, line); // skip header
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        boost::split(row, line, boost::is_any_of(","));
        xs.emplace_back(boost::lexical_cast<double>(row[0]));
        ys.emplace_back(boost::lexical_cast<double>(row[1]));
    }
}

template<typename T>
T lerp(T a, T b, T t) {
    return a + (b - a) * t;
}

void write_to_csv(std::vector<double>const &smoothed_x, 
                  std::vector<double>const &smoothed_y,
                  std::vector<double>const &heading, 
                  std::vector<double>const &kappa, 
                  std::vector<double>const &dkappa)
{
  std::ofstream outFile;
  outFile.open("smoothed_path.csv", std::ios::out);
  outFile << "smoothed_x" << ',' << "smoothed_y" << ',' << "heading" << ',' 
          << "kappa" << ',' << "dkappa" << '\n';
  for (size_t i = 0; i < smoothed_x.size(); ++i)
  {
    outFile << smoothed_x.at(i) << ',' << smoothed_y.at(i) << ','
            << heading.at(i) << ',' << kappa.at(i) << ',' << dkappa.at(i) << '\n';
  }
  outFile.close();
}


int main(int argc, char **argv)
{
  constexpr double kSegmentLength = 0.5;
  constexpr double kMathEps = 0.0001;

  std::string exec_path = argv[0];
  boost::filesystem::path path(exec_path);
  std::string data_path = path.parent_path().append("../map1_2023-4-18-16-19-36.csv").string();
  std::cout << data_path << std::endl;

  std::vector<double> x_s, y_s, acc_s;
  x_s.reserve(1000);
  y_s.reserve(1000);
  acc_s.reserve(1000);
  read_data(data_path, x_s, y_s);
  acc_s.emplace_back(0);
  for (size_t i = 1; i < x_s.size(); i++) {
      auto s = acc_s.back() + std::hypot(x_s[i] - x_s[i - 1], y_s[i] - y_s[i - 1]);
      acc_s.emplace_back(s);
  }
  auto total_length = acc_s.back();
  int segment_size = std::ceil(total_length / kSegmentLength);
  auto segment_slice = Eigen::VectorXd::LinSpaced(segment_size, 0, total_length);
  std::vector<double> x_s_sliced, y_s_sliced;
  x_s_sliced.resize(segment_slice.size());
  y_s_sliced.resize(segment_slice.size());
  auto lower = acc_s.begin();
  for (Eigen::Index i = 0; i < segment_slice.size(); i++) {
      auto target_s = segment_slice[i];
      auto upper = std::upper_bound(lower, acc_s.end(), target_s);

      if (upper == acc_s.end()) {
          x_s_sliced[i] = x_s.back();
          y_s_sliced[i] = y_s.back();
          break;
      }
      auto upper_index = std::distance(acc_s.begin(), upper);
      auto prev_upper = upper - 1;
      auto den = *upper - *prev_upper;
      auto t = den > kMathEps ? (target_s - *prev_upper) / den : 0;
      x_s_sliced[i] = lerp(x_s[upper_index - 1], x_s[upper_index], t);
      y_s_sliced[i] = lerp(y_s[upper_index - 1], y_s[upper_index], t);
      lower = prev_upper;
  }
  std::vector<std::pair<double, double>> raw_path;
  std::pair<double, double> raw_point;
  for (size_t i = 0; i < x_s_sliced.size(); ++i)
  {
    raw_point = {x_s_sliced[i], y_s_sliced[i]};
    raw_path.emplace_back(raw_point);
  }
  std::cout << "raw_path.size: " << raw_path.size() << std::endl;

  ReferenceLineSmootherConfig config;
  auto smoother = std::make_shared<DiscretePointsReferenceLineSmoother>(config);  

  std::vector<ReferencePoint> smoothed_reference_line;
  auto start_time = std::chrono::system_clock::now();
  smoother->Smooth(raw_path, &smoothed_reference_line);
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  std::cout << "Time for solving = " << diff.count() * 1000 << " ms.\n";

  std::vector<double> smoothed_x, smoothed_y, heading, kappa, dkappa;
  for (const auto &it : smoothed_reference_line) {
    smoothed_x.emplace_back(it.x());
    smoothed_y.emplace_back(it.y());
    heading.emplace_back(it.heading());
    kappa.emplace_back(it.kappa());
    dkappa.emplace_back(it.dkappa());
  }

  write_to_csv(smoothed_x, smoothed_y, heading, kappa, dkappa);

  namespace plt = matplotlibcpp;

  plt::named_plot("Interpolated", x_s_sliced, y_s_sliced, "b,");
  plt::named_plot("Smoothed", smoothed_x, smoothed_y, "r,");
  plt::axis("equal");
  plt::xlabel("x/m");
  plt::ylabel("y/m");
  plt::title("Trajectory");
  plt::legend();
  plt::save("Compare.svg");

  return 0;
}