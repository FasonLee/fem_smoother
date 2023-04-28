/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "include/discrete_points_reference_line_smoother.h"
#include "include/fem_pos_deviation_smoother.h"
#include "include/discrete_points_math.h"

namespace apollo
{
  namespace planning
  {
    DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother(
        const ReferenceLineSmootherConfig &config)
        : ReferenceLineSmoother(config) {}
        
    bool DiscretePointsReferenceLineSmoother::Smooth(
        const std::vector<std::pair<double, double>>& raw_path,
        std::vector<ReferencePoint>* smoothed_reference_line)
    {
      std::vector<std::pair<double, double>> raw_point2d;
      std::vector<double> anchorpoints_lateralbound;

      for (const auto &point : raw_path)
      {
        raw_point2d.emplace_back(point);
        anchorpoints_lateralbound.emplace_back(0.25);
      }

      // fix front and back points to avoid end states deviate from the center of
      // road
      anchorpoints_lateralbound.front() = 0.0;
      anchorpoints_lateralbound.back() = 0.0;

      NormalizePoints(&raw_point2d);

      std::vector<std::pair<double, double>> smoothed_point2d;

      auto status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
                              &smoothed_point2d);
      if (!status)
      {
        std::cout << "discrete_points reference line smoother fails" << std::endl;
        return false;
      }

      DeNormalizePoints(&smoothed_point2d);

      std::vector<ReferencePoint> ref_points;
      GenerateRefPointProfile(smoothed_point2d, &ref_points);

      ReferencePoint::RemoveDuplicates(&ref_points);

      if (ref_points.size() < 2)
      {
        std::cout << "Fail to generate smoothed reference line." << std::endl;
        return false;
      }

      for (const auto &item : ref_points)
      {
        smoothed_reference_line->emplace_back(item);
      }
      return true;
    }

    bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
        const std::vector<std::pair<double, double>> &raw_point2d,
        const std::vector<double> &bounds,
        std::vector<std::pair<double, double>> *ptr_smoothed_point2d)
    {
      const auto &fem_pos_config =
          config_.discrete_points().fem_pos_deviation_smoothing();

      FemPosDeviationSmoother smoother(fem_pos_config);

      // box contraints on pos are used in fem pos smoother, thus shrink the
      // bounds by 1.0 / sqrt(2.0)
      std::vector<double> box_bounds = bounds;
      const double box_ratio = 1.0 / std::sqrt(2.0);
      for (auto &bound : box_bounds)
      {
        bound *= box_ratio;
      }

      std::vector<double> opt_x;
      std::vector<double> opt_y;
      bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

      if (!status)
      {
        std::cout << "Fem Pos reference line smoothing failed" << std::endl;
        return false;
      }
      if (opt_x.size() < 2 || opt_y.size() < 2)
      {
        std::cout << "Return by fem pos smoother is wrong. Size smaller than 2 " << std::endl;
        return false;
      }
      size_t point_size = opt_x.size();
      for (size_t i = 0; i < point_size; ++i)
      {
        ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
      }
      return true;
    }

    void DiscretePointsReferenceLineSmoother::NormalizePoints(
        std::vector<std::pair<double, double>> *xy_points)
    {
      zero_x_ = xy_points->front().first;
      zero_y_ = xy_points->front().second;
      std::for_each(xy_points->begin(), xy_points->end(),
                    [this](std::pair<double, double> &point)
                    {
                      auto curr_x = point.first;
                      auto curr_y = point.second;
                      std::pair<double, double> xy(curr_x - zero_x_,
                                                   curr_y - zero_y_);
                      point = std::move(xy);
                    });
    }

    void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
        std::vector<std::pair<double, double>> *xy_points)
    {
      std::for_each(xy_points->begin(), xy_points->end(),
                    [this](std::pair<double, double> &point)
                    {
                      auto curr_x = point.first;
                      auto curr_y = point.second;
                      std::pair<double, double> xy(curr_x + zero_x_,
                                                   curr_y + zero_y_);
                      point = std::move(xy);
                    });
    }

    bool DiscretePointsReferenceLineSmoother::GenerateRefPointProfile(
        const std::vector<std::pair<double, double>> &xy_points,
        std::vector<ReferencePoint> *ref_path)
    {
      // Compute path profile
      std::vector<double> headings;
      std::vector<double> kappas;
      std::vector<double> dkappas;
      std::vector<double> accumulated_s;
      if (!DiscretePointsMath::ComputePathProfile(
              xy_points, &headings, &accumulated_s, &kappas, &dkappas))
      {
        return false;
      }

      // Load into ReferencePoints
      size_t points_size = xy_points.size();
      ReferencePoint ref_point;
      for (size_t i = 0; i < points_size; ++i)
      {
        ref_point = {xy_points[i].first, xy_points[i].second,
                     headings[i], kappas[i], dkappas[i]};
                     
        ref_path->emplace_back(ref_point);
      }

      return true;
    }
  } // namespace planning
} // namespace apollo
