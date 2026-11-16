// Invoke the real Karto rasterizer. Exit nonzero if any valid endpoint is lost.
// Build against the selected SLAM prefix, not a copied/mock implementation.
#include <karto_sdk/Karto.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

struct Scenario {
  const char *name;
  double x, y, heading, range, resolution;
};

int main() {
  const karto::Name name("livingmap_extent_regression");
  std::unique_ptr<karto::LaserRangeFinder> laser(
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name));
  laser->SetMinimumRange(.0001);
  laser->SetMaximumRange(12.);
  laser->SetRangeThreshold(12.);
  laser->SetMinimumAngle(-M_PI);
  laser->SetMaximumAngle(M_PI);
  laser->SetAngularResolution(M_PI / 2.);
  karto::SensorManager::GetInstance()->RegisterSensor(laser.get());
  const Scenario scenarios[] = {
      {"axis_aligned", 0., 0., 0., 1., .05},
      {"positive_translation", 8.137, 4.219, 0., 1., .05},
      {"negative_translation", -18.137, -7.219, 0., 1., .05},
      {"rotated_diagonals", .123, -.456, M_PI / 4., 1., .05},
      {"rotated_arbitrary", -2.17, 3.31, .371, 1.013, .05},
      {"nonintegral_extent", 0., 0., 0., 1.013, .05},
      {"coarser_resolution", -.23, -.18, .21, 1.017, .1},
      {"finer_resolution", 1.78, -4.12, -.52, 1.023, .02},
  };
  unsigned failed = 0;
  for (const auto &test : scenarios) {
    std::vector<std::unique_ptr<karto::LocalizedRangeScan>> owned;
    karto::LocalizedRangeScanVector scans;
    for (int i = 0; i < 4; ++i) {
      auto scan = std::make_unique<karto::LocalizedRangeScan>(
          name, karto::RangeReadingsVector(5, test.range));
      scan->SetCorrectedPose(karto::Pose2(test.x, test.y, test.heading));
      scan->SetOdometricPose(karto::Pose2(test.x, test.y, test.heading));
      scans.push_back(scan.get());
      owned.push_back(std::move(scan));
    }
    std::unique_ptr<karto::OccupancyGrid> grid(
        karto::OccupancyGrid::CreateFromScans(scans, test.resolution, 2, .1));
    bool ok = true;
    unsigned occupied = 0;
    // Check every returned hit, on positive and negative boundaries alike.
    for (const auto &point : scans.front()->GetPointReadings(false)) {
      auto index = grid->WorldToGrid(point);
      bool valid = grid->IsValidGridIndex(index);
      bool hit = valid && grid->GetValue(index) == karto::GridStates_Occupied;
      occupied += hit;
      ok = ok && hit;
      if (valid) {
        auto restored = grid->GridToWorld(index);
        ok = ok && std::abs(restored.GetX() - point.GetX()) <= test.resolution / 2. + 1e-10
                && std::abs(restored.GetY() - point.GetY()) <= test.resolution / 2. + 1e-10;
      }
    }
    auto minimum = scans.front()->GetBoundingBox().GetMinimum();
    auto offset = grid->GetCoordinateConverter()->GetOffset();
    ok = ok && std::abs(offset.GetX() - minimum.GetX()) < 1e-10
            && std::abs(offset.GetY() - minimum.GetY()) < 1e-10;
    auto center = grid->WorldToGrid(karto::Vector2<kt_double>(test.x, test.y));
    ok = ok && grid->IsValidGridIndex(center)
            && grid->GetValue(center) == karto::GridStates_Free;
    std::cout << test.name << " " << (ok ? "PASS" : "FAIL")
              << " retained_hits=" << occupied << "/5"
              << " size=" << grid->GetWidth() << "x" << grid->GetHeight() << "\n";
    failed += !ok;
  }
  karto::SensorManager::GetInstance()->UnregisterSensor(laser.get());
  std::cout << "8 scenarios, " << failed << " failed\n";
  return failed ? 1 : 0;
}
