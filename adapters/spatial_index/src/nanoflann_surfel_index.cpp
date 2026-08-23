#include "adapters/spatial_index/nanoflann_surfel_index.hpp"

#include <nanoflann.hpp>

namespace uw::adapters::spatial_index {

// Zero-copy: nanoflann only ever stores integer ids, reading coordinates
// back through this adaptor on demand. `points` must outlive every Tree
// built against this adaptor — satisfied here since both are
// NanoflannSurfelIndex members with the adaptor constructed to reference
// the (never-reassigned, only ever cleared-in-place) `points_` vector.
struct NanoflannSurfelIndex::PointCloudAdaptor {
  const std::vector<Eigen::Vector3d>& points;
  explicit PointCloudAdaptor(const std::vector<Eigen::Vector3d>& points_ref) : points(points_ref) {}

  std::size_t kdtree_get_point_count() const { return points.size(); }
  double kdtree_get_pt(std::size_t idx, std::size_t dim) const { return points[idx](static_cast<int>(dim)); }
  template <class Bbox>
  bool kdtree_get_bbox(Bbox&) const {
    return false;
  }
};

NanoflannSurfelIndex::NanoflannSurfelIndex() {
  adaptor_ = std::make_unique<PointCloudAdaptor>(points_);
  tree_ = std::make_unique<Tree>(/*dimensionality=*/3, *adaptor_);
}

// Defined here (not defaulted in the header) because Tree/PointCloudAdaptor
// are incomplete types at the point the header is included elsewhere —
// unique_ptr's destructor needs the complete type, which only this
// translation unit (after #include <nanoflann.hpp>) has.
NanoflannSurfelIndex::~NanoflannSurfelIndex() = default;

std::uint32_t NanoflannSurfelIndex::AppendPoint(const Eigen::Vector3d& position, std::size_t logical_index) {
  points_.push_back(position);
  const auto internal_id = static_cast<std::uint32_t>(points_.size() - 1);
  internal_to_logical_.push_back(logical_index);  // parallels points_ 1:1, always grows together
  return internal_id;
}

void NanoflannSurfelIndex::Clear() {
  points_.clear();
  internal_to_logical_.clear();
  logical_to_internal_.clear();
  // Rebuild the tree fresh against the same, now-empty `points_` — the
  // adaptor's reference stays valid (points_ is cleared in place, never
  // reassigned to a new vector object).
  tree_ = std::make_unique<Tree>(/*dimensionality=*/3, *adaptor_);
}

void NanoflannSurfelIndex::NotifyInserted(std::size_t index, const Eigen::Vector3d& position_W) {
  const std::uint32_t internal_id = AppendPoint(position_W, index);
  if (logical_to_internal_.size() <= index) logical_to_internal_.resize(index + 1);
  logical_to_internal_[index] = internal_id;
  tree_->addPoint(internal_id);
}

void NanoflannSurfelIndex::NotifyMoved(std::size_t index, const Eigen::Vector3d& new_position_W) {
  // Deliberately does NOT reuse the old nanoflann-internal id: nanoflann's
  // removePoint is a lazy tombstone (the node's OLD coordinates stay live
  // in the tree structure for pruning purposes until a rebuild physically
  // drops it), so re-adding under the SAME id without a rebuild would risk
  // stale geometry. Tombstoning the old id and allocating a fresh one for
  // the new position is always correct, at the cost of relying on
  // nanoflann's own bounded-garbage rebuild (KDTreeIncrementalIndexParams'
  // alpha_deleted, default 0.5) to reclaim tombstones over time rather than
  // managing that ourselves — SurfelMap's workload is merge-heavy, so this
  // path is hit often; see this class's header doc comment for why the
  // OTHER nanoflann dynamic adaptor (a static-subtree forest) was rejected
  // for exactly this reason.
  const std::uint32_t old_internal_id = logical_to_internal_.at(index);
  tree_->removePoint(old_internal_id);
  const std::uint32_t new_internal_id = AppendPoint(new_position_W, index);
  logical_to_internal_[index] = new_internal_id;
  tree_->addPoint(new_internal_id);
}

void NanoflannSurfelIndex::NotifyRemovedBySwapPop(std::size_t index, std::size_t previous_back_index) {
  const std::uint32_t removed_internal_id = logical_to_internal_.at(index);
  tree_->removePoint(removed_internal_id);
  if (index != previous_back_index) {
    const std::uint32_t moved_internal_id = logical_to_internal_.at(previous_back_index);
    logical_to_internal_[index] = moved_internal_id;
    internal_to_logical_[moved_internal_id] = index;  // that surfel's logical identity changed too
  }
  logical_to_internal_.resize(previous_back_index);
}

std::optional<std::size_t> NanoflannSurfelIndex::FindNearestWithinRadius(const Eigen::Vector3d& query,
                                                                          double radius_m) const {
  if (points_.empty()) return std::nullopt;
  const double query_pt[3] = {query.x(), query.y(), query.z()};
  const double radius_sq = radius_m * radius_m;

  // Two nanoflann quirks stacked here, both load-bearing for exactness:
  //
  // 1. Every distance/radius parameter for an L2 metric is the SQUARED
  //    Euclidean distance (RadiusResultSet/RKNNResultSet compare against
  //    un-square-rooted accumulated distances) — passing a plain radius is
  //    a well-known footgun.
  // 2. RadiusResultSet::addPoint tests `dist < radius` — STRICTLY less
  //    than — so a point at EXACTLY radius_m is silently excluded, unlike
  //    SurfelMap's own brute-force FindNearest (`distance <= best_distance`
  //    — inclusive). Caught by
  //    NanoflannSurfelIndex.FindNearestRespectsExactRadiusBoundaryNotItsSquare.
  //    Fixed by querying a hair past the true radius (never changes which
  //    point is nearest — it can only ever ADD candidates strictly farther
  //    than radius_m to the tail of the sorted result list) and then
  //    re-checking every candidate against the EXACT (non-inflated)
  //    radius_sq ourselves before accepting it — an `rknnSearch(k=1)`
  //    instead would risk this rejection discarding the single globally-
  //    nearest candidate even when a different, slightly farther point IS
  //    within the true radius, so this uses radiusSearch's full sorted
  //    list and picks the first entry that passes the exact check.
  const double inflated_radius_sq = radius_sq * (1.0 + 1e-9) + 1e-12;
  std::vector<nanoflann::ResultItem<std::uint32_t, double>> matches;
  static_cast<void>(tree_->radiusSearch(query_pt, inflated_radius_sq, matches));  // sorted ascending by distance
  for (const auto& match : matches) {
    if (match.second <= radius_sq) return internal_to_logical_.at(match.first);
  }
  return std::nullopt;
}

std::vector<std::size_t> NanoflannSurfelIndex::FindCandidatesNearSegment(const Eigen::Vector3d& segment_start,
                                                                          const Eigen::Vector3d& segment_end,
                                                                          double radius_m) const {
  if (points_.empty()) return {};
  // Enclosing-ball over-approximation, guaranteed to contain every point
  // within radius_m of the segment (triangle inequality: any such point is
  // within half_length + radius_m of the segment's midpoint) — see
  // SurfelSpatialIndex's header doc comment: over-approximating is always
  // safe, CarveFreeSpace's own exact math is the narrow-phase filter.
  const Eigen::Vector3d midpoint = (segment_start + segment_end) * 0.5;
  const double half_length = (segment_end - segment_start).norm() * 0.5;
  const double query_radius = half_length + radius_m;
  const double query_pt[3] = {midpoint.x(), midpoint.y(), midpoint.z()};

  std::vector<nanoflann::ResultItem<std::uint32_t, double>> matches;
  // Squared (nanoflann convention) and inflated by the same tiny epsilon
  // margin as FindNearestWithinRadius, for the same reason (nanoflann's
  // radius comparison is strict `<`) — over-approximation is already this
  // method's contract, so the margin only ever adds a few more harmless
  // candidates for CarveFreeSpace's own exact math to filter back out.
  const double query_radius_sq = query_radius * query_radius * (1.0 + 1e-9) + 1e-12;
  static_cast<void>(tree_->radiusSearch(query_pt, query_radius_sq, matches));

  std::vector<std::size_t> out;
  out.reserve(matches.size());
  for (const auto& match : matches) out.push_back(internal_to_logical_.at(match.first));
  return out;
}

}  // namespace uw::adapters::spatial_index
