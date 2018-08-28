#pragma once

#include "drake/geometry/identifier.h"

namespace drake {
namespace geometry {

<<<<<<< HEAD
/** Type used to identify geometry sources in SceneGraph. */
using SourceId = Identifier<class SourceTag>;

/** Type used to identify geometry frames in SceneGraph.*/
using FrameId = Identifier<class FrameTag>;

/** Type used to identify geometry instances in SceneGraph. */
=======
/** Type used to identify geometry sources in GeometrySystem. */
using SourceId = Identifier<class SourceTag>;

/** Type used to identify geometry frames in GeometrySystem .*/
using FrameId = Identifier<class FrameTag>;

/** Type used to identify geometry instances in GeometrySystem. */
>>>>>>> intial
using GeometryId = Identifier<class GeometryTag>;

}  // namespace geometry
}  // namespace drake
