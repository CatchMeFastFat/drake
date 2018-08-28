#include "drake/multibody/shapes/visual_element.h"

<<<<<<< HEAD
#include <string>

=======
>>>>>>> intial
namespace DrakeShapes {

VisualElement::VisualElement(const Eigen::Isometry3d& T_element_to_local_in)
    : Element(T_element_to_local_in),
      material(Eigen::Vector4d(0.7, 0.7, 0.7, 1)) {}

VisualElement::VisualElement(const Geometry& geometry_in,
                             const Eigen::Isometry3d& T_element_to_local_in,
<<<<<<< HEAD
                             const Eigen::Vector4d& material_in,
                             const std::string& name_in)
    : Element(geometry_in, T_element_to_local_in), material(material_in),
      name(name_in) {}
=======
                             const Eigen::Vector4d& material_in)
    : Element(geometry_in, T_element_to_local_in), material(material_in) {}
>>>>>>> intial

void VisualElement::setMaterial(const Eigen::Vector4d& material_in) {
  material = material_in;
}

const Eigen::Vector4d& VisualElement::getMaterial() const {
  return material;
}

}  // namespace DrakeShapes
