#include "drake/geometry/query_object.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_context.h"
<<<<<<< HEAD
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
=======
#include "drake/geometry/geometry_system.h"
>>>>>>> intial

namespace drake {
namespace geometry {

// Helper class for testing the query object.
class QueryObjectTester {
 public:
  QueryObjectTester() = delete;

  template <typename T>
  static std::unique_ptr<QueryObject<T>> MakeQueryObject() {
    return std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
  }

  template <typename T>
  static std::unique_ptr<QueryObject<T>> MakeQueryObject(
<<<<<<< HEAD
      const GeometryContext<T>* context, const SceneGraph<T>* scene_graph) {
    auto q = std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
    q->set(context, scene_graph);
=======
      const GeometryContext<T>* context, const GeometrySystem<T>* system) {
    auto q = std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
    q->system_ = system;
    q->context_ = context;
>>>>>>> intial
    return q;
  }

  template <typename T>
  static void expect_default(const QueryObject<T>& object) {
<<<<<<< HEAD
    EXPECT_EQ(object.scene_graph_, nullptr);
=======
    EXPECT_EQ(object.system_, nullptr);
>>>>>>> intial
    EXPECT_EQ(object.context_, nullptr);
  }

  template <typename T>
  static void expect_live(const QueryObject<T>& object) {
<<<<<<< HEAD
    EXPECT_NE(object.scene_graph_, nullptr);
=======
    EXPECT_NE(object.system_, nullptr);
>>>>>>> intial
    EXPECT_NE(object.context_, nullptr);
  }

  template <typename T>
  static void ThrowIfDefault(const QueryObject<T>& object) {
    object.ThrowIfDefault();
  }
};

namespace {

<<<<<<< HEAD
using std::make_unique;
=======
>>>>>>> intial
using std::unique_ptr;
using systems::Context;

class QueryObjectTest : public ::testing::Test {
 protected:
  using QOT = QueryObjectTester;

  void SetUp() override {
<<<<<<< HEAD
    context_ = scene_graph_.AllocateContext();
    geom_context_ = dynamic_cast<GeometryContext<double>*>(context_.get());
    ASSERT_NE(geom_context_, nullptr);
    query_object_ = QOT::MakeQueryObject(geom_context_, &scene_graph_);
=======
    context_ = system_.AllocateContext();
    geom_context_ = dynamic_cast<GeometryContext<double>*>(context_.get());
    ASSERT_NE(geom_context_, nullptr);
    query_object_ = QOT::MakeQueryObject(geom_context_, &system_);
>>>>>>> intial

    QueryObjectTester::expect_live(*query_object_);
  }

<<<<<<< HEAD
  SceneGraph<double> scene_graph_;
=======
  GeometrySystem<double> system_;
>>>>>>> intial
  unique_ptr<Context<double>> context_;
  GeometryContext<double>* geom_context_{nullptr};
  unique_ptr<QueryObject<double>> query_object_;
};

// Confirm copy semantics.
TEST_F(QueryObjectTest, CopySemantics) {
  // Default query object *can* be copied and assigned.
  unique_ptr<QueryObject<double>> default_object =
      QOT::MakeQueryObject<double>();
  QOT::expect_default(*default_object);

  QueryObject<double> from_default{*default_object};
  QOT::expect_default(from_default);

  QueryObject<double> from_live{*query_object_};
  QOT::expect_default(from_live);
}

<<<<<<< HEAD
// NOTE: This doesn't test the specific queries; GeometryQuery simply wraps
// the class (SceneGraph) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows.
=======
>>>>>>> intial
TEST_F(QueryObjectTest, DefaultQueryThrows) {
  unique_ptr<QueryObject<double>> default_object =
      QOT::MakeQueryObject<double>();
  QOT::expect_default(*default_object);

#define EXPECT_DEFAULT_ERROR(expression) \
  DRAKE_EXPECT_THROWS_MESSAGE(expression, std::runtime_error, \
      "Attempting to perform query on invalid QueryObject.+");

  EXPECT_DEFAULT_ERROR(QOT::ThrowIfDefault(*default_object));

  // Enumerate *all* queries to confirm they throw the proper exception.
<<<<<<< HEAD
  EXPECT_DEFAULT_ERROR(default_object->ComputePointPairPenetration());
  EXPECT_DEFAULT_ERROR(
      default_object->ComputeSignedDistancePairwiseClosestPoints());
=======
  EXPECT_DEFAULT_ERROR(default_object->GetFrameId(GeometryId::get_new_id()));
  EXPECT_DEFAULT_ERROR(default_object->GetSourceName(SourceId::get_new_id()));
  EXPECT_DEFAULT_ERROR(default_object->ComputePointPairPenetration());
>>>>>>> intial

#undef EXPECT_DEFAULT_ERROR
}

<<<<<<< HEAD
// Confirms the inspector returned by the QueryObject is "correct" (in that
// it accesses the correct state).
GTEST_TEST(QueryObjectInspectTest, CreateValidInspector) {
  SceneGraph<double> scene_graph;
  SourceId source_id = scene_graph.RegisterSource("source");
  auto identity = Isometry3<double>::Identity();
  FrameId frame_id =
      scene_graph.RegisterFrame(source_id, GeometryFrame("frame", identity));
  GeometryId geometry_id = scene_graph.RegisterGeometry(
      source_id, frame_id, make_unique<GeometryInstance>(
                               identity, make_unique<Sphere>(1.0), "sphere"));
  unique_ptr<Context<double>> context = scene_graph.AllocateContext();
  auto geo_context = dynamic_cast<GeometryContext<double>*>(context.get());
  unique_ptr<QueryObject<double>> query_object =
      QueryObjectTester::MakeQueryObject<double>(geo_context, &scene_graph);

  const SceneGraphInspector<double>& inspector = query_object->inspector();

  // Perform a single query to confirm that the inspector has access to the
  // state uniquely populated above (guaranteed via the uniqueness of frame and
  // geometry identifiers).
  EXPECT_EQ(inspector.GetFrameId(geometry_id), frame_id);
}
=======
// NOTE: This doesn't test the specific queries; GeometryQuery simply wraps
// the class (GeometrySystem) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows.
>>>>>>> intial

}  // namespace
}  // namespace geometry
}  // namespace drake
