#include "drake/geometry/geometry_instance.h"

<<<<<<< HEAD
#include "drake/geometry/utilities.h"
=======
#include <utility>
>>>>>>> intial

namespace drake {
namespace geometry {

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
<<<<<<< HEAD
                                   std::unique_ptr<Shape> shape,
                                   const std::string& name)
    : GeometryInstance(X_PG, std::move(shape), name, VisualMaterial()) {}

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
                                   std::unique_ptr<Shape> shape,
                                   const std::string& name,
=======
                                   std::unique_ptr<Shape> shape)
    : GeometryInstance(X_PG, std::move(shape), VisualMaterial()) {}

GeometryInstance::GeometryInstance(const Isometry3<double>& X_PG,
                                   std::unique_ptr<Shape> shape,
>>>>>>> intial
                                   const VisualMaterial& vis_material)
    : id_(GeometryId::get_new_id()),
      X_PG_(X_PG),
      shape_(std::move(shape)),
<<<<<<< HEAD
      name_(detail::CanonicalizeStringName(name)),
      visual_material_(vis_material) {
  if (name_.empty()) {
    throw std::logic_error("GeometryInstance given the name '" + name +
                           "' which is an empty canonical string");
  }
}
=======
      visual_material_(vis_material) {}
>>>>>>> intial

}  // namespace geometry
}  // namespace drake
