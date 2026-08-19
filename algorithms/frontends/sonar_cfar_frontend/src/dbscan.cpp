#include "uw/frontends/dbscan.hpp"

#include <queue>

namespace uw::frontends {

namespace {
std::vector<int> RegionQuery(const std::vector<Eigen::Vector2d>& points, int index, double eps) {
  std::vector<int> neighbors;
  const double eps_sq = eps * eps;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if ((points[i] - points[index]).squaredNorm() <= eps_sq) neighbors.push_back(i);
  }
  return neighbors;
}
}  // namespace

std::vector<int> Dbscan(const std::vector<Eigen::Vector2d>& points, double eps, int min_samples) {
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -2);  // -2 = unvisited, -1 = noise, >=0 = cluster id
  int next_cluster = 0;

  for (int i = 0; i < n; ++i) {
    if (labels[i] != -2) continue;

    auto neighbors = RegionQuery(points, i, eps);
    if (static_cast<int>(neighbors.size()) < min_samples) {
      labels[i] = -1;
      continue;
    }

    const int cluster_id = next_cluster++;
    labels[i] = cluster_id;

    std::queue<int> to_visit;
    for (int neighbor : neighbors) to_visit.push(neighbor);

    while (!to_visit.empty()) {
      const int current = to_visit.front();
      to_visit.pop();

      if (labels[current] == -1) labels[current] = cluster_id;  // was noise, now border point
      if (labels[current] != -2) continue;                      // already visited/assigned

      labels[current] = cluster_id;
      auto current_neighbors = RegionQuery(points, current, eps);
      if (static_cast<int>(current_neighbors.size()) >= min_samples) {
        for (int neighbor : current_neighbors) {
          if (labels[neighbor] == -2 || labels[neighbor] == -1) to_visit.push(neighbor);
        }
      }
    }
  }

  return labels;
}

}  // namespace uw::frontends
