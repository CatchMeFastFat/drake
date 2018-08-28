#include "drake/manipulation/dev/remote_tree_viewer_wrapper.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

#include "nlohmann/json.hpp"

#include "drake/lcmt_viewer2_comms.hpp"

using nlohmann::json;

namespace drake {
namespace manipulation {
namespace dev {
static double GetUnixTime(void) {
  return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

<<<<<<< HEAD
json MakePointCloud(const Eigen::Matrix3Xd& pts,
                    const std::vector<std::vector<double>>& color) {
  json j = {{"type", "pointcloud"},
            {"points", std::vector<std::vector<double>>()},
            {"channels", {{"rgb", std::vector<std::vector<double>>()}}}};

  // Push in the points and colors.
  for (int i = 0; i < pts.cols(); i++) {
    j["points"].push_back({pts(0, i), pts(1, i), pts(2, i)});
    if (color.size() == 1)
      j["channels"]["rgb"].push_back(color[0]);
    else if (static_cast<int>(color.size()) == pts.cols())
      j["channels"]["rgb"].push_back(color[i]);
=======
RemoteTreeViewerWrapper::RemoteTreeViewerWrapper() {}

void FillLcmView2CommsMessage(int64_t now, const json& j,
                              lcmt_viewer2_comms* msg) {
  msg->utime = now;
  msg->format = "treeviewer_json";
  msg->format_version_major = 1;
  msg->format_version_minor = 0;
  msg->data.clear();
  const std::string j_dump = j.dump();
  std::copy(j_dump.begin(), j_dump.end(), std::back_inserter(msg->data));
  msg->num_bytes = j_dump.size();
}

void RemoteTreeViewerWrapper::PublishPointCloud(
    const Eigen::Matrix3Xd& pts, const std::vector<std::string>& path,
    const std::vector<std::vector<double>>& color) {
  int64_t now = GetUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {
      {"timestamp", now},
      {
          "setgeometry",
          {{{"path", path},
            {"geometry",
             {{"type", "pointcloud"},
              {"points", std::vector<std::vector<double>>()},
              {"channels", {{"rgb", std::vector<std::vector<double>>()}}}}}}},
      },
      {"settransform", json({})},
      {"delete", json({})}};

  // Push in the points and colors.
  for (int i = 0; i < pts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["points"].push_back(
        {pts(0, i), pts(1, i), pts(2, i)});
    if (color.size() == 1)
      j["setgeometry"][0]["geometry"]["channels"]["rgb"].push_back(color[0]);
    else if (static_cast<int>(color.size()) == pts.cols())
      j["setgeometry"][0]["geometry"]["channels"]["rgb"].push_back(color[i]);
>>>>>>> intial
    else if (color.size() != 0)
      printf("Color does not have right number of entries: %ld vs %ld\n",
             color.size(), pts.cols());
  }
<<<<<<< HEAD
  return j;
}

json MakeLine(const Eigen::Matrix3Xd& pts,
              const Eigen::Ref<const Eigen::Vector4d>& color) {
  json j = {
      {"type", "line"},
      {"points", std::vector<std::vector<double>>()},
      {"color", {color(0), color(1), color(2), color(3)}},
  };

  // Push in the points.
  for (int i = 0; i < pts.cols(); i++) {
    j["points"].push_back({pts(0, i), pts(1, i), pts(2, i)});
  }
  return j;
}

json MakeArrow(const Eigen::Ref<const Eigen::Vector3d>& start,
               const Eigen::Ref<const Eigen::Vector3d>& end, double radius,
               double head_radius, double head_length,
               const Eigen::Ref<const Eigen::Vector4d>& color) {
  return {
      {"type", "line"},
      {"points", {{start(0), start(1), start(2)}, {end(0), end(1), end(2)}}},
      {"radius", radius},
      {"end_head", true},
      {"head_radius", head_radius},
      {"head_length", head_length},
      {"color", {color(0), color(1), color(2), color(3)}},
  };
}

json MakeMeshData(const Eigen::Matrix3Xd& verts,
                  const std::vector<Eigen::Vector3i>& tris) {
  json j = {{"type", "mesh_data"},
            {"vertices", std::vector<std::vector<double>>()},
            {"faces", std::vector<std::vector<int>>()}};

  // Push in the points.
  for (int i = 0; i < verts.cols(); i++) {
    j["vertices"].push_back({verts(0, i), verts(1, i), verts(2, i)});
  }

  // Push in the triangle indices.
  for (int i = 0; i < static_cast<int>(tris.size()); i++) {
    j["faces"].push_back({tris[i][0], tris[i][1], tris[i][2]});
  }

  return j;
}

json MakeDrakeShapesGeometry(const DrakeShapes::Geometry& geometry,
                             const Eigen::Vector4d& color) {
  json j;
  j["color"] = {color[0], color[1], color[2], color[3]};
  switch (geometry.getShape()) {
    case DrakeShapes::BOX: {
      const DrakeShapes::Box* box =
          static_cast<const DrakeShapes::Box*>(&geometry);
      j["type"] = std::string("box");
      j["lengths"] =
          std::vector<double>({box->size[0], box->size[1], box->size[2]});
    } break;
    case DrakeShapes::SPHERE: {
      const DrakeShapes::Sphere* sphere =
          static_cast<const DrakeShapes::Sphere*>(&geometry);
      j["type"] = std::string("sphere");
      j["radius"] = sphere->radius;
    } break;
    case DrakeShapes::CYLINDER: {
      const DrakeShapes::Cylinder* cylinder =
          static_cast<const DrakeShapes::Cylinder*>(&geometry);
      j["type"] = std::string("cylinder");
      j["radius"] = cylinder->radius;
      j["length"] = cylinder->length;
    } break;
    case DrakeShapes::MESH: {
      const DrakeShapes::Mesh* mesh =
          static_cast<const DrakeShapes::Mesh*>(&geometry);
      j["type"] = std::string("mesh_file");
      j["filename"] = mesh->resolved_filename_;
      j["scale"] = std::vector<double>(
          {mesh->scale_[0], mesh->scale_[1], mesh->scale_[2]});
    } break;
    case DrakeShapes::CAPSULE: {
      const DrakeShapes::Capsule* capsule =
          static_cast<const DrakeShapes::Capsule*>(&geometry);
      j["type"] = std::string("capsule");
      j["radius"] = capsule->radius;
      j["length"] = capsule->length;
    } break;
    case DrakeShapes::MESH_POINTS: {
      // Since DrakeShapes::MeshPoints doesn't implement getFaces(), we just
      // display the points here.
      const DrakeShapes::MeshPoints* mesh_points =
          static_cast<const DrakeShapes::MeshPoints*>(&geometry);
      Eigen::Matrix3Xd points;
      mesh_points->getPoints(points);
      return MakePointCloud(points, {{color[0], color[1], color[2]}});
    } break;
    default:
      std::cout << "Unsupported geometry type " << geometry.getShape()
                << ", sorry!" << std::endl;
      return json();
  }
  return j;
}

json MakeSetGeometryElement(const std::vector<std::string>& path,
                            const json geometry) {
  return {{"path", path}, {"geometry", geometry}};
}

json MakeSetGeometryElementForDrakeShapesGeometry(
    const DrakeShapes::Geometry& geometry, const Eigen::Vector4d& color,
    const std::vector<std::string>& path) {
  return MakeSetGeometryElement(path, MakeDrakeShapesGeometry(geometry, color));
}

json MakeSetTransformElement(const Eigen::Affine3d& tf,
                             const std::vector<std::string>& path) {
  // Extract std::vector-formatted translation and quaternion from the tf
  std::vector<double> translation = {tf.translation()[0], tf.translation()[1],
                                     tf.translation()[2]};
  Eigen::Quaterniond q(tf.rotation());
  std::vector<double> rotation = {q.w(), q.x(), q.y(), q.z()};

  // Format a JSON string to update the transform for @p path
  // clang-format off
  return {{"path", path},
          {"transform", {
            {"translation", translation},
            {"quaternion", rotation}}}};
  // clang-format on
}

json::array_t MakeSetGeometryArrayForRigidBody(
    const RigidBodyTree<double>& tree, int body_index,
    const Eigen::Vector4d& color, const std::vector<std::string>& path,
    bool visual) {
  json::array_t setgeometry = json::array();
  const auto& body = tree.get_body(body_index);
  if (visual) {
    int ctr = 0;
    for (const auto& element : body.get_visual_elements()) {
      if (element.hasGeometry()) {
        std::vector<std::string> full_name = path;
        full_name.push_back(body.get_name());
        full_name.push_back("visual");
        full_name.push_back(std::to_string(ctr++));
        setgeometry.push_back(MakeSetGeometryElementForDrakeShapesGeometry(
            element.getGeometry(), color, full_name));
      }
    }
  } else {
    int ctr = 0;
    for (const auto& collision_elem_id : body.get_collision_element_ids()) {
      auto element = tree.FindCollisionElement(collision_elem_id);
      if (element->hasGeometry()) {
        std::vector<std::string> full_name = path;
        full_name.push_back(body.get_name());
        full_name.push_back("collision");
        full_name.push_back(std::to_string(ctr++));
        setgeometry.push_back(MakeSetGeometryElementForDrakeShapesGeometry(
            element->getGeometry(), color, full_name));
      }
    }
  }
  return setgeometry;
}

json::array_t MakeSetGeometryArrayForRigidBodyTree(
    const RigidBodyTree<double>& tree, const Eigen::Vector4d& color,
    const std::vector<std::string>& path, bool visual) {
  json::array_t setgeometry = json::array();
  for (int i = 0; i < tree.get_num_bodies(); ++i) {
    json::array_t body_setgeometry =
        MakeSetGeometryArrayForRigidBody(tree, i, color, path, visual);
    setgeometry.insert(setgeometry.end(), body_setgeometry.begin(),
                       body_setgeometry.end());
  }
  return setgeometry;
}

json MakeSetTransformArrayForRigidBody(const RigidBodyTree<double>& tree,
                                       int body_index,
                                       const Eigen::Affine3d& X_WB,
                                       const std::vector<std::string>& path,
                                       bool visual) {
  json::array_t settransform = json::array();
  const auto& body = tree.get_body(body_index);
  if (visual) {
=======

  auto msg = lcmt_viewer2_comms();
  FillLcmView2CommsMessage(now, j, &msg);
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::PublishLine(
    const Eigen::Matrix3Xd& pts, const std::vector<std::string>& path) {
  int64_t now = GetUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {
                       {"type", "line"},
                       {"points", std::vector<std::vector<double>>()},
                   }}}},
            },
            {"settransform", json({})},
            {"delete", json({})}};

  // Push in the points and colors.
  for (int i = 0; i < pts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["points"].push_back(
        {pts(0, i), pts(1, i), pts(2, i)});
  }

  auto msg = lcmt_viewer2_comms();
  FillLcmView2CommsMessage(now, j, &msg);
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::PublishArrow(
    const Eigen::Ref<const Eigen::Vector3d>& start,
    const Eigen::Ref<const Eigen::Vector3d>& end,
    const std::vector<std::string>& path, double radius, double head_radius,
    double head_length, const Eigen::Ref<const Eigen::Vector4d>& color) {
  int64_t now = GetUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {
                       {"type", "line"},
                       {"points", std::array<std::array<double, 3>, 2>()},
                       {"color", std::array<double, 4>()},
                   }}}},
            },
            {"settransform", json({})},
            {"delete", json({})}};

  // Push in the points and colors.
  j["setgeometry"][0]["geometry"]["points"][0] = {start(0), start(1), start(2)};
  j["setgeometry"][0]["geometry"]["points"][1] = {end(0), end(1), end(2)};
  j["setgeometry"][0]["geometry"]["radius"] = radius;
  j["setgeometry"][0]["geometry"]["end_head"] = true;
  j["setgeometry"][0]["geometry"]["head_radius"] = head_radius;
  j["setgeometry"][0]["geometry"]["head_length"] = head_length;
  j["setgeometry"][0]["geometry"]["color"] = {color(0), color(1), color(2),
                                              color(3)};

  auto msg = lcmt_viewer2_comms();
  FillLcmView2CommsMessage(now, j, &msg);
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}
void RemoteTreeViewerWrapper::PublishRawMesh(
    const Eigen::Matrix3Xd& verts, const std::vector<Eigen::Vector3i>& tris,
    const std::vector<std::string>& path) {
  int64_t now = GetUnixTime() * 1000 * 1000;
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {{"type", "mesh_data"},
                    {"vertices", std::vector<std::vector<double>>()},
                    {"faces", std::vector<std::vector<int>>()}}}}},
            },
            {"settransform", json({})},
            {"delete", json({})}};

  // Push in the points.
  for (int i = 0; i < verts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["vertices"].push_back(
        {verts(0, i), verts(1, i), verts(2, i)});
  }
  // Push in the triangle indices.
  for (int i = 0; i < static_cast<int>(tris.size()); i++) {
    j["setgeometry"][0]["geometry"]["faces"].push_back(
        {tris[i][0], tris[i][1], tris[i][2]});
  }

  auto msg = lcmt_viewer2_comms();
  FillLcmView2CommsMessage(now, j, &msg);
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::PublishRigidBodyTree(
    const RigidBodyTree<double>& tree, const Eigen::VectorXd& q,
    const Eigen::Vector4d& color, const std::vector<std::string>& path,
    bool visual) {
  auto kinematics_cache = tree.doKinematics(q);
  for (int i = 0; i < tree.get_num_bodies(); ++i) {
    PublishRigidBody(
        tree, i,
        tree.relativeTransform(kinematics_cache, 0, i),
        color, path, visual);
  }
}

void RemoteTreeViewerWrapper::PublishRigidBody(
    const RigidBodyTree<double>& tree, int body_index,
    const Eigen::Affine3d& tf, const Eigen::Vector4d& color,
    const std::vector<std::string>& path, bool visual_flag) {
  const auto& body = tree.get_body(body_index);
  if (visual_flag) {
>>>>>>> intial
    int ctr = 0;
    for (const auto& element : body.get_visual_elements()) {
      if (element.hasGeometry()) {
        std::vector<std::string> full_name = path;
<<<<<<< HEAD
        full_name.push_back(body.get_name());
        full_name.push_back("visual");
        full_name.push_back(std::to_string(ctr++));
        settransform.push_back(MakeSetTransformElement(
            X_WB * element.getLocalTransform(), full_name));
=======
        full_name.push_back(body.get_name() + "_visual_" +
                            std::to_string(ctr++));
        PublishGeometry(element.getGeometry(), tf * element.getLocalTransform(),
                        color, full_name);
>>>>>>> intial
      }
    }
  } else {
    int ctr = 0;
    for (const auto& collision_elem_id : body.get_collision_element_ids()) {
      auto element = tree.FindCollisionElement(collision_elem_id);
      if (element->hasGeometry()) {
        std::vector<std::string> full_name = path;
<<<<<<< HEAD
        full_name.push_back(body.get_name());
        full_name.push_back("collision");
        full_name.push_back(std::to_string(ctr++));
        settransform.push_back(MakeSetTransformElement(
            X_WB * element->getLocalTransform(), full_name));
      }
    }
  }
  return settransform;
}

json::array_t MakeSetTransformArrayForRigidBodyTree(
    const RigidBodyTree<double>& tree, const Eigen::VectorXd& q,
    const std::vector<std::string>& path, bool visual) {
  json::array_t settransform = json::array();
  auto kinematics_cache = tree.doKinematics(q);
  for (int i = 0; i < tree.get_num_bodies(); ++i) {
    json::array_t body_settransform = MakeSetTransformArrayForRigidBody(
        tree, i, tree.relativeTransform(kinematics_cache, 0, i), path, visual);
    settransform.insert(settransform.end(), body_settransform.begin(),
                        body_settransform.end());
  }
  return settransform;
}

void FillLcmView2CommsMessage(int64_t now, const json& j,
                              lcmt_viewer2_comms* msg) {
  msg->utime = now;
  msg->format = "treeviewer_json";
  msg->format_version_major = 1;
  msg->format_version_minor = 0;
  msg->data.clear();
  const std::string j_dump = j.dump();
  std::copy(j_dump.begin(), j_dump.end(), std::back_inserter(msg->data));
  msg->num_bytes = j_dump.size();
}

void PublishRemoteTreeViewerMessage(json::array_t setgeometry,
                                    json::array_t settransform,
                                    json::array_t delete_array,
                                    drake::lcm::DrakeLcm* lcm) {
  int64_t now = GetUnixTime() * 1000 * 1000;
  json j = {{"timestamp", now},
            {"setgeometry", setgeometry},
            {"settransform", settransform},
            {"delete", delete_array}};
  auto msg = lcmt_viewer2_comms();
  FillLcmView2CommsMessage(now, j, &msg);
  // Use channel 0 for remote viewer communications.
  lcm->get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

RemoteTreeViewerWrapper::RemoteTreeViewerWrapper(drake::lcm::DrakeLcm* lcm) {
  if (lcm == nullptr) {
    owned_lcm_ = std::make_unique<drake::lcm::DrakeLcm>();
    lcm_ = owned_lcm_.get();
  } else {
    lcm_ = lcm;
  }
}

void RemoteTreeViewerWrapper::PublishPointCloud(
    const Eigen::Matrix3Xd& pts, const std::vector<std::string>& path,
    const std::vector<std::vector<double>>& color) {
  PublishRemoteTreeViewerMessage(
      {MakeSetGeometryElement(path, MakePointCloud(pts, color))},
      json::array() /* settransform */, json::array() /* delete_array */, lcm_);
}

void RemoteTreeViewerWrapper::PublishLine(
    const Eigen::Matrix3Xd& pts, const std::vector<std::string>& path,
    const Eigen::Ref<const Eigen::Vector4d>& color) {
  PublishRemoteTreeViewerMessage(
      {MakeSetGeometryElement(path, MakeLine(pts, color))},
      json::array() /* settransform */, json::array() /* delete_array */, lcm_);
}

void RemoteTreeViewerWrapper::PublishArrow(
    const Eigen::Ref<const Eigen::Vector3d>& start,
    const Eigen::Ref<const Eigen::Vector3d>& end,
    const std::vector<std::string>& path, double radius, double head_radius,
    double head_length, const Eigen::Ref<const Eigen::Vector4d>& color) {
  PublishRemoteTreeViewerMessage(
      {MakeSetGeometryElement(path, MakeArrow(start, end, radius, head_radius,
                                              head_length, color))},
      json::array() /* settransform */, json::array() /* delete_array */, lcm_);
}

void RemoteTreeViewerWrapper::PublishRawMesh(
    const Eigen::Matrix3Xd& verts, const std::vector<Eigen::Vector3i>& tris,
    const std::vector<std::string>& path) {
  PublishRemoteTreeViewerMessage(
      {MakeSetGeometryElement(path, MakeMeshData(verts, tris))},
      json::array() /* settransform */, json::array() /* delete_array */, lcm_);
}

void RemoteTreeViewerWrapper::PublishRigidBodyTree(
    const RigidBodyTree<double>& tree, const Eigen::VectorXd& q,
    const Eigen::Vector4d& color, const std::vector<std::string>& path,
    bool visual) {
  auto delete_array = json::array({json({})});
  delete_array.front()["path"] = path;
  PublishRemoteTreeViewerMessage(json::array() /* setgeometry */,
                                 json::array() /* settransform */, delete_array,
                                 lcm_);
  PublishRemoteTreeViewerMessage(
      MakeSetGeometryArrayForRigidBodyTree(tree, color, path, visual),
      MakeSetTransformArrayForRigidBodyTree(tree, q, path, visual),
      json::array() /* delete_array */, lcm_);
}

void RemoteTreeViewerWrapper::UpdateRigidBodyTree(
    const RigidBodyTree<double>& tree, const Eigen::VectorXd& q,
    const std::vector<std::string>& path, bool visual) {
  PublishRemoteTreeViewerMessage(
      json::array() /* setgeometry */,
      MakeSetTransformArrayForRigidBodyTree(tree, q, path, visual),
      json::array() /* delete_array */, lcm_);
}

void RemoteTreeViewerWrapper::PublishRigidBody(
    const RigidBodyTree<double>& tree, int body_index,
    const Eigen::Affine3d& tf, const Eigen::Vector4d& color,
    const std::vector<std::string>& path, bool visual) {
  PublishRemoteTreeViewerMessage(
      MakeSetGeometryArrayForRigidBody(tree, body_index, color, path, visual),
      MakeSetTransformArrayForRigidBody(tree, body_index, tf, path, visual),
      json::array() /* delete_array */, lcm_);
}

void RemoteTreeViewerWrapper::PublishGeometry(
    const DrakeShapes::Geometry& geometry, const Eigen::Affine3d& tf,
    const Eigen::Vector4d& color, const std::vector<std::string>& path) {
  PublishRemoteTreeViewerMessage(
      {MakeSetGeometryElementForDrakeShapesGeometry(geometry, color, path)},
      {MakeSetTransformElement(tf, path)}, json::array() /* delete_array */,
      lcm_);
}

=======
        full_name.push_back(body.get_name() + "_collision_" +
                            std::to_string(ctr++));
        PublishGeometry(element->getGeometry(),
                        tf * element->getLocalTransform(), color, full_name);
      }
    }
  }
}

void RemoteTreeViewerWrapper::PublishGeometry(
    const DrakeShapes::Geometry& geometry, const Eigen::Affine3d& tf,
    const Eigen::Vector4d& color, const std::vector<std::string>& path) {
  int64_t now = GetUnixTime() * 1000 * 1000;

  // Short-circuit to points if the passed geometry is a set of mesh points
  if (geometry.getShape() == DrakeShapes::MESH_POINTS) {
    Eigen::Matrix3Xd pts;
    geometry.getPoints(pts);
    pts = tf * pts;
    return PublishPointCloud(pts, path, {{color[0], color[1], color[2]}});
  }

  // Extract std::vector-formatted translation and quaternion from the tf
  std::vector<double> translation = {tf.translation()[0], tf.translation()[1],
                                     tf.translation()[2]};
  Eigen::Quaterniond q(tf.rotation());
  std::vector<double> rotation = {q.w(), q.x(), q.y(), q.z()};

  // Format a JSON string for this bit of geometry (in a setgeometry call)
  // and its transformation (in a settransform call)
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {{"color", {color[0], color[1], color[2], color[3]}}}}}},
            },
            {"settransform",
             {{{"path", path},
               {"transform",
                {{"translation", translation}, {"quaternion", rotation}}}}}},
            {"delete", json({})}};

  // Fill in the setgeometry call based on what kind of geometry this is
  switch (geometry.getShape()) {
    case DrakeShapes::BOX: {
      const DrakeShapes::Box* box =
          static_cast<const DrakeShapes::Box*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("box");
      j["setgeometry"][0]["geometry"]["lengths"] =
          std::vector<double>({box->size[0], box->size[1], box->size[2]});
    } break;
    case DrakeShapes::SPHERE: {
      const DrakeShapes::Sphere* sphere =
          static_cast<const DrakeShapes::Sphere*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("sphere");
      j["setgeometry"][0]["geometry"]["radius"] = sphere->radius;
    } break;
    case DrakeShapes::CYLINDER: {
      const DrakeShapes::Cylinder* cylinder =
          static_cast<const DrakeShapes::Cylinder*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("cylinder");
      j["setgeometry"][0]["geometry"]["radius"] = cylinder->radius;
      j["setgeometry"][0]["geometry"]["length"] = cylinder->length;
    } break;
    case DrakeShapes::MESH: {
      const DrakeShapes::Mesh* mesh =
          static_cast<const DrakeShapes::Mesh*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("mesh_file");
      j["setgeometry"][0]["geometry"]["filename"] = mesh->resolved_filename_;
      j["setgeometry"][0]["geometry"]["scale"] = std::vector<double>(
          {mesh->scale_[0], mesh->scale_[1], mesh->scale_[2]});
    } break;
    case DrakeShapes::CAPSULE: {
      const DrakeShapes::Capsule* capsule =
          static_cast<const DrakeShapes::Capsule*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("capsule");
      j["setgeometry"][0]["geometry"]["radius"] = capsule->radius;
      j["setgeometry"][0]["geometry"]["length"] = capsule->length;
    } break;
    default:
      std::cout << "Unsupported geometry type " << geometry.getShape()
                << ", sorry!" << std::endl;
      return;
  }

  auto msg = lcmt_viewer2_comms();
  FillLcmView2CommsMessage(now, j, &msg);
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}
>>>>>>> intial
}  // namespace dev
}  // namespace manipulation
}  // namespace drake
