// nanoflann-backed implementation of uw::mapping::SurfelSpatialIndex — see
// that interface's doc comment (include/mapping/surfel_map.hpp) for the
// Notify*/query contract this must satisfy exactly. Lives outside
// include/ and src/ (mirrors adapters/ros2's precedent, see
// tools/lint/check_layer_dependencies.py) specifically so nanoflann.hpp
// never has to be visible from `mapping`, keeping SurfelMap's own headers
// third-party-type-free. Only `application` (which the layer-dependency
// rule allows to depend on everything) is meant to construct this and
// inject it into a SurfelMap.
//
// docs/archive/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md
// §6.2/§5.2 is the design this implements — nanoflann accelerates
// SurfelMap's existing FindNearest/CarveFreeSpace scans, it does not
// change SurfelMap's semantics or replace its Surfel representation.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "mapping/surfel_map.hpp"

// Forward-declared rather than #included here: nanoflann.hpp is ~4000 lines
// of template machinery and only the .cpp needs to instantiate it — keeps
// this header (and anything that includes it) cheap to compile.
namespace nanoflann {
template <typename T, class DataSource, typename DistanceType, typename IndexType>
struct L2_Simple_Adaptor;
template <typename Distance, class DatasetAdaptor, std::int32_t DIM, typename IndexType>
class KDTreeSingleIndexIncrementalAdaptor;
}  // namespace nanoflann

namespace uw::adapters::spatial_index {

// Wraps nanoflann::KDTreeSingleIndexIncrementalAdaptor — a single
// weight-balanced KD-tree with bounded-garbage lazy deletion (subtrees
// physically rebuild once their tombstoned fraction crosses an internal
// threshold), chosen specifically because SurfelMap's workload is
// move-heavy (every confidence-weighted merge relocates a surfel) rather
// than append-only: nanoflann's OTHER dynamic adaptor
// (KDTreeSingleIndexDynamicAdaptor, a "logarithmic forest" of static
// subtrees) reactivates a removed index's OLD geometry in place if the
// same index number is reused, which would silently search stale
// coordinates for a moved point — this class sidesteps that entirely by
// never reusing a nanoflann-internal point id for NotifyMoved (see the
// .cpp: a move always tombstones the old id and allocates a fresh one).
class NanoflannSurfelIndex : public uw::mapping::SurfelSpatialIndex {
 public:
  NanoflannSurfelIndex();
  ~NanoflannSurfelIndex() override;

  NanoflannSurfelIndex(const NanoflannSurfelIndex&) = delete;
  NanoflannSurfelIndex& operator=(const NanoflannSurfelIndex&) = delete;

  void Clear() override;
  void NotifyInserted(std::size_t index, const Eigen::Vector3d& position_W) override;
  void NotifyMoved(std::size_t index, const Eigen::Vector3d& new_position_W) override;
  void NotifyRemovedBySwapPop(std::size_t index, std::size_t previous_back_index) override;
  std::optional<std::size_t> FindNearestWithinRadius(const Eigen::Vector3d& query,
                                                       double radius_m) const override;
  std::vector<std::size_t> FindCandidatesNearSegment(const Eigen::Vector3d& segment_start,
                                                       const Eigen::Vector3d& segment_end,
                                                       double radius_m) const override;

 private:
  // Zero-copy dataset adaptor nanoflann reads point coordinates through —
  // defined in the .cpp (needs nanoflann's adaptor concept, not forward-
  // declarable usefully). Points physically live in `points_` below;
  // nanoflann only ever stores integer ids into it.
  struct PointCloudAdaptor;

  using Metric = nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor, double, std::uint32_t>;
  using Tree = nanoflann::KDTreeSingleIndexIncrementalAdaptor<Metric, PointCloudAdaptor, 3, std::uint32_t>;

  std::uint32_t AppendPoint(const Eigen::Vector3d& position, std::size_t logical_index);

  std::vector<Eigen::Vector3d> points_;                 // nanoflann-internal id -> position (append-only)
  std::vector<std::size_t> internal_to_logical_;         // nanoflann-internal id -> current logical surfel index
  std::vector<std::uint32_t> logical_to_internal_;       // logical surfel index -> current nanoflann-internal id
  std::unique_ptr<PointCloudAdaptor> adaptor_;
  std::unique_ptr<Tree> tree_;
};

}  // namespace uw::adapters::spatial_index
