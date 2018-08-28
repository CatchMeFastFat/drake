#include "drake/geometry/internal_geometry.h"

namespace drake {
namespace geometry {
namespace internal {

InternalGeometry::InternalGeometry() : InternalGeometryBase() {}

InternalGeometry::InternalGeometry(std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
<<<<<<< HEAD
                                   const std::string& name,
                                   const Isometry3<double>& X_PG,
                                   GeometryIndex engine_index,
                                   const optional<GeometryId>& parent_id)
    : InternalGeometry(std::move(shape), frame_id, geometry_id, name, X_PG,
=======
                                   const Isometry3<double>& X_PG,
                                   GeometryIndex engine_index,
                                   const optional<GeometryId>& parent_id)
    : InternalGeometry(std::move(shape), frame_id, geometry_id, X_PG,
>>>>>>> intial
                       engine_index, VisualMaterial(), parent_id) {}

InternalGeometry::InternalGeometry(std::unique_ptr<Shape> shape,
                                   FrameId frame_id, GeometryId geometry_id,
<<<<<<< HEAD
                                   const std::string& name,
=======
>>>>>>> intial
                                   const Isometry3<double>& X_PG,
                                   GeometryIndex engine_index,
                                   const VisualMaterial& vis_material,
                                   const optional<GeometryId>& parent_id)
<<<<<<< HEAD
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_PG,
                           vis_material),
=======
    : InternalGeometryBase(std::move(shape), geometry_id, X_PG, vis_material),
>>>>>>> intial
      frame_id_(frame_id),
      engine_index_(engine_index),
      parent_id_(parent_id) {}

InternalAnchoredGeometry::InternalAnchoredGeometry() : InternalGeometryBase() {}

InternalAnchoredGeometry::InternalAnchoredGeometry(
    std::unique_ptr<Shape> shape, GeometryId geometry_id,
<<<<<<< HEAD
    const std::string& name, const Isometry3<double> X_WG,
    AnchoredGeometryIndex engine_index)
    : InternalAnchoredGeometry(std::move(shape), geometry_id, name, X_WG,
=======
    const Isometry3<double>& X_WG, AnchoredGeometryIndex engine_index)
    : InternalAnchoredGeometry(std::move(shape), geometry_id, X_WG,
>>>>>>> intial
                               engine_index, VisualMaterial()) {}

InternalAnchoredGeometry::InternalAnchoredGeometry(
    std::unique_ptr<Shape> shape, GeometryId geometry_id,
<<<<<<< HEAD
    const std::string& name, const Isometry3<double> X_WG,
    AnchoredGeometryIndex engine_index, const VisualMaterial& vis_material)
    : InternalGeometryBase(std::move(shape), geometry_id, name, X_WG,
                           vis_material),
=======
    const Isometry3<double>& X_WG, AnchoredGeometryIndex engine_index,
    const VisualMaterial& vis_material)
    : InternalGeometryBase(std::move(shape), geometry_id, X_WG, vis_material),
>>>>>>> intial
      engine_index_(engine_index) {}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
