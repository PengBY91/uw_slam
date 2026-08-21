#pragma once

#include <vector>

#include <Eigen/Core>

namespace uw::frontends {

// Standard textbook DBSCAN (Ester et al. 1996) over 2D points, O(n^2)
// neighbor queries — fine for the small per-frame scanline point counts
// here. This is an original reimplementation, not a port: sonar_camera_
// reconstruction's cluster_scanline wraps sklearn's DBSCAN, which this
// repository does not depend on (see NOTICE — only the CFAR/imaging_sonar
// geometry was ported from that repository, not its clustering
// dependency). Returns one label per input point: -1 = noise, otherwise a
// 0-based cluster index.
std::vector<int> Dbscan(const std::vector<Eigen::Vector2d>& points, double eps, int min_samples);

}  // namespace uw::frontends
