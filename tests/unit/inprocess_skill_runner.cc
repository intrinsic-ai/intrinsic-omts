// Copyright 2026 Intrinsic Innovation LLC

#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/initialize.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "incode/motion_planning/skills/move_robot.h"
#include "incode/motion_planning/skills/preplan_motion.h"
#include "incode/motion_planning/skills/preplan_motion.pb.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/storage/dummy_storage.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/motion_planner_client.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.grpc.pb.h"
#include "intrinsic/motion_planning/service/motion_planner_service_in_process.h"
#include "intrinsic/motion_planning/skills/move_robot.pb.h"
#include "intrinsic/resources/proto/geometric_resource_data.pb.h"
#include "intrinsic/resources/proto/resource_handle.pb.h"
#include "intrinsic/scene/proto/v1/scene_object.pb.h"
#include "intrinsic/skills/apps/attach_object_to_robot.h"
#include "intrinsic/skills/apps/attach_object_to_robot.pb.h"
#include "intrinsic/skills/apps/detach_object.h"
#include "intrinsic/skills/apps/detach_object.pb.h"
#include "intrinsic/skills/apps/update_world.pb.h"
#include "intrinsic/skills/cc/equipment_pack.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/skills/testing/skill_test_utils.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"
#include "intrinsic/world/service/test/world_service_fake.h"
#include "intrinsic/world/service/world_service.grpc.pb.h"
#include "intrinsic/world/util/compose_world_util.h"

namespace intrinsic {
namespace {

constexpr absl::string_view kWorldId = "world";

template <typename T>
absl::StatusOr<T> ReadTextProtoFile(absl::string_view path) {
  std::ifstream ifs{std::string(path)};
  if (!ifs.is_open()) {
    return absl::NotFoundError(
        absl::StrCat("Could not open textproto file: ", path));
  }
  std::ostringstream ss;
  ss << ifs.rdbuf();
  return ParseTextProto<T>(ss.str());
}

std::string JsonEscape(absl::string_view input) {
  return absl::StrReplaceAll(input, {{"\\", "\\\\"},
                                     {"\"", "\\\""},
                                     {"\n", "\\n"},
                                     {"\r", "\\r"},
                                     {"\t", "\\t"}});
}

std::string StatusToJson(const absl::Status& status) {
  if (status.ok()) {
    return "{\"ok\":true,\"code\":0,\"message\":\"\"}";
  }
  std::string message(status.message());
  status.ForEachPayload(
      [&message](absl::string_view /*type_url*/, const absl::Cord& payload) {
        std::string flat = std::string(payload);
        std::string printable;
        printable.reserve(flat.size());
        for (unsigned char c : flat) {
          if (c >= 32 && c < 127) {
            printable.push_back(static_cast<char>(c));
          } else {
            printable.push_back(' ');
          }
        }
        absl::StrAppend(&message, " ", printable);
      });
  return absl::StrCat("{\"ok\":false,\"code\":", status.raw_code(),
                      ",\"message\":\"", JsonEscape(message), "\"}");
}

bool HasExcludedCollisionRule(
    const intrinsic_proto::world::CollisionSettings& settings,
    absl::string_view obj_a, absl::string_view obj_b) {
  for (const auto& rule : settings.collision_rules()) {
    if (!rule.collision_action().is_excluded()) {
      continue;
    }
    auto matches = [](const auto& refs, absl::string_view name) {
      for (const auto& n : refs) {
        if (n.has_object() && n.object().by_name().object_name() == name) {
          return true;
        }
        if (n.has_object_with_filter() &&
            n.object_with_filter().reference().by_name().object_name() ==
                name) {
          return true;
        }
      }
      return false;
    };
    if ((matches(rule.left(), obj_a) && matches(rule.right(), obj_b)) ||
        (matches(rule.left(), obj_b) && matches(rule.right(), obj_a))) {
      return true;
    }
  }
  return false;
}

// Returns the entity filter to use when attaching an object to `gripper`
// (matching `GetAttachmentEntityFilter` in
// `intrinsic/skills/apps/attach_object_to_robot.cc`).
world::ObjectEntityFilter GetAttachmentEntityFilter(
    const world::WorldObject& gripper) {
  std::optional<world::KinematicObject> kinematic_object =
      world::KinematicObject::FromTransformNode(gripper);
  if (kinematic_object.has_value() &&
      !kinematic_object->IsoFlangeFrameIds().empty()) {
    return world::ObjectEntityFilter().IncludeFinalEntity();
  }
  return world::ObjectEntityFilter().IncludeBaseEntity();
}

// Returns the local name of the entity in `parent` to which `child`'s root
// entity is directly attached (matching `GetAttachedParentEntityName` in
// `intrinsic/skills/apps/attach_object_to_robot_test.cc`).
absl::StatusOr<std::string> GetAttachedParentEntityName(
    const world::WorldObject& parent, const world::WorldObject& child) {
  const auto& child_entities = child.Proto().entities();
  auto child_root_it = child_entities.find(child.Proto().root_entity_id());
  if (child_root_it == child_entities.end()) {
    return absl::NotFoundError("Child root entity not found in child proto.");
  }
  const std::string& parent_entity_id = child_root_it->second.parent_id();
  const auto& parent_entities = parent.Proto().entities();
  auto parent_entity_it = parent_entities.find(parent_entity_id);
  if (parent_entity_it == parent_entities.end()) {
    return absl::NotFoundError(absl::StrCat("Parent entity ", parent_entity_id,
                                            " not found in parent proto."));
  }
  return parent_entity_it->second.name();
}

class InProcessSkillRunner {
 public:
  absl::StatusOr<std::string> InitWorld(absl::string_view scene_objects_arg,
                                        absl::string_view updates_arg) {
    std::vector<intrinsic_proto::resources::GeometricResourceInstanceData>
        instances;
    for (absl::string_view item :
         absl::StrSplit(scene_objects_arg, ';', absl::SkipEmpty())) {
      std::pair<absl::string_view, absl::string_view> kv =
          absl::StrSplit(item, absl::MaxSplits('=', 1));
      if (kv.first.empty() || kv.second.empty()) {
        return absl::InvalidArgumentError(
            absl::StrCat("Invalid scene object spec: ", item));
      }
      INTR_ASSIGN_OR_RETURN(
          auto scene_object,
          ReadTextProtoFile<intrinsic_proto::scene_object::v1::SceneObject>(
              kv.second));
      intrinsic_proto::resources::GeometricResourceInstanceData instance;
      instance.set_name(std::string(kv.first));
      *instance.mutable_scene_object() = std::move(scene_object);
      instances.push_back(std::move(instance));
    }

    intrinsic_proto::world::ObjectWorldUpdates merged_updates;
    for (absl::string_view path :
         absl::StrSplit(updates_arg, ';', absl::SkipEmpty())) {
      INTR_ASSIGN_OR_RETURN(
          auto file_updates,
          ReadTextProtoFile<intrinsic_proto::world::ObjectWorldUpdates>(path));
      merged_updates.mutable_updates()->MergeFrom(file_updates.updates());
    }

    auto dummy_geolib = GetDummyGeometryLibrary();
    ComposeWorldOptions compose_options;
    compose_options.quiet = true;
    INTR_ASSIGN_OR_RETURN(
        ComposeWorldResult composed,
        ComposeResourceInstancesIntoWorld(instances, merged_updates,
                                          *dummy_geolib, compose_options));

    FakeWorldService::CreateOptions fake_options;
    fake_options.enable_local_tcp = true;
    INTR_ASSIGN_OR_RETURN(world_service_,
                          FakeWorldService::Create(fake_options));
    INTR_RETURN_IF_ERROR(
        world_service_->AddWorld(kWorldId, std::move(composed.world)));

    world_service_stub_ = world_service_->NewStub();
    object_world_service_stub_ = world_service_->NewObjectStub();
    world_client_ = std::make_unique<world::ObjectWorldClient>(
        std::string(kWorldId), object_world_service_stub_);

    motion_planner_service_ = MotionPlannerServiceInProcess::Create(
        world_service_stub_.get(), object_world_service_stub_.get(),
        world_service_->GetGeometryLibrary());
    motion_planner_stub_ = motion_planner_service_->GetStub();
    motion_planner_client_ =
        std::make_unique<motion_planning::MotionPlannerClient>(
            std::string(kWorldId), motion_planner_stub_);
    skill_test_factory_ = std::make_unique<skills::SkillTestFactory>(
        world_service_->GetGeometryLibrary());

    INTR_ASSIGN_OR_RETURN(std::string address, world_service_->GetAddress());
    return absl::StrCat("{\"ok\":true,\"code\":0,\"address\":\"",
                        JsonEscape(address), "\"}");
  }

  std::string AttachObject(absl::string_view b64_params) {
    if (!world_client_) {
      return StatusToJson(
          absl::FailedPreconditionError("World not initialized"));
    }
    std::string raw_bytes;
    if (!absl::Base64Unescape(b64_params, &raw_bytes)) {
      return StatusToJson(absl::InvalidArgumentError("Invalid base64 payload"));
    }
    intrinsic_proto::skills::AttachObjectToRobotParams params;
    if (!params.ParseFromString(raw_bytes)) {
      return StatusToJson(absl::InvalidArgumentError(
          "Failed to parse AttachObjectToRobotParams"));
    }

    absl::Status status =
        skills::AttachObjectToRobotSkill::ExecuteImpl(params, *world_client_)
            .status();
    if (!status.ok()) {
      status = ApplyEntityFilterAttach(params);
      if (!status.ok()) {
        return StatusToJson(status);
      }
    }

    absl::StatusOr<world::WorldObject> gripper =
        world_client_->GetObject(params.gripper_entity());
    if (!gripper.ok()) {
      return StatusToJson(gripper.status());
    }
    absl::StatusOr<world::WorldObject> object =
        world_client_->GetObject(params.object_entity());
    if (!object.ok()) {
      return StatusToJson(object.status());
    }
    absl::StatusOr<std::string> attached_entity =
        GetAttachedParentEntityName(*gripper, *object);
    if (!attached_entity.ok()) {
      return StatusToJson(attached_entity.status());
    }
    absl::StatusOr<intrinsic_proto::world::CollisionSettings>
        collision_settings = world_client_->GetCollisionSettings();
    if (!collision_settings.ok()) {
      return StatusToJson(collision_settings.status());
    }
    bool collisions_disabled = HasExcludedCollisionRule(
        *collision_settings, gripper->Name().value(), object->Name().value());

    return absl::StrCat("{\"ok\":true,\"code\":0,\"parent_object\":\"",
                        JsonEscape(object->ParentName().value()),
                        "\",\"parent_entity\":\"", JsonEscape(*attached_entity),
                        "\",\"collisions_disabled\":",
                        collisions_disabled ? "true" : "false", "}");
  }

  std::string DetachObject(absl::string_view b64_params) {
    if (!world_client_) {
      return StatusToJson(
          absl::FailedPreconditionError("World not initialized"));
    }
    std::string raw_bytes;
    if (!absl::Base64Unescape(b64_params, &raw_bytes)) {
      return StatusToJson(absl::InvalidArgumentError("Invalid base64 payload"));
    }
    intrinsic_proto::skills::DetachObjectParams params;
    if (!params.ParseFromString(raw_bytes)) {
      return StatusToJson(
          absl::InvalidArgumentError("Failed to parse DetachObjectParams"));
    }
    absl::Status status =
        skills::DetachObjectSkill::ExecuteImpl(params, *world_client_).status();
    if (!status.ok()) {
      return StatusToJson(status);
    }
    absl::StatusOr<intrinsic_proto::world::CollisionSettings>
        collision_settings = world_client_->GetCollisionSettings();
    if (!collision_settings.ok()) {
      return StatusToJson(collision_settings.status());
    }
    bool collisions_disabled = HasExcludedCollisionRule(
        *collision_settings, params.gripper_entity().by_name().object_name(),
        params.object_entity().by_name().object_name());
    return absl::StrCat("{\"ok\":true,\"code\":0,\"collisions_disabled\":",
                        collisions_disabled ? "true" : "false", "}");
  }

  std::string UpdateWorld(absl::string_view b64_params) {
    if (!world_client_) {
      return StatusToJson(
          absl::FailedPreconditionError("World not initialized"));
    }
    std::string raw_bytes;
    if (!absl::Base64Unescape(b64_params, &raw_bytes)) {
      return StatusToJson(absl::InvalidArgumentError("Invalid base64 payload"));
    }
    intrinsic_proto::skills::UpdateWorldParams params;
    if (!params.ParseFromString(raw_bytes)) {
      return StatusToJson(
          absl::InvalidArgumentError("Failed to parse UpdateWorldParams"));
    }
    if (params.has_update()) {
      intrinsic_proto::world::ObjectWorldUpdates updates;
      *updates.add_updates() = params.update();
      return StatusToJson(world_client_->BatchUpdate(updates));
    }
    if (params.has_updates()) {
      return StatusToJson(world_client_->BatchUpdate(params.updates()));
    }
    return StatusToJson(absl::OkStatus());
  }

  std::string MoveRobot(absl::string_view robot_name,
                        absl::string_view b64_params) {
    if (!world_client_ || !motion_planner_client_) {
      return StatusToJson(
          absl::FailedPreconditionError("World not initialized"));
    }
    std::string raw_bytes;
    if (!absl::Base64Unescape(b64_params, &raw_bytes)) {
      return StatusToJson(absl::InvalidArgumentError("Invalid base64 payload"));
    }
    intrinsic_proto::skills::MoveRobotParams params;
    if (!params.ParseFromString(raw_bytes)) {
      return StatusToJson(
          absl::InvalidArgumentError("Failed to parse MoveRobotParams"));
    }

    absl::StatusOr<world::KinematicObject> robot =
        world_client_->GetKinematicObject(
            WorldObjectName(std::string(robot_name)));
    if (!robot.ok()) {
      return StatusToJson(robot.status());
    }

    auto plan_result = skills::MoveRobot::PlanTrajectory(
        *robot, *motion_planner_client_, params, std::nullopt,
        "inprocess_skill_runner");
    if (!plan_result.ok()) {
      return StatusToJson(plan_result.status());
    }
    if (plan_result->trajectory.state().empty()) {
      return StatusToJson(
          absl::InternalError("Planned trajectory contained no states"));
    }

    const auto& start_state = plan_result->trajectory.state().at(0);
    const auto& end_state = plan_result->trajectory.state().at(
        plan_result->trajectory.state().size() - 1);
    eigenmath::VectorXd end_state_vec =
        RepeatedDoubleToVectorXd(end_state.position());
    absl::Status update_status =
        world_client_->UpdateJointPositions(*robot, end_state_vec);
    if (!update_status.ok()) {
      return StatusToJson(update_status);
    }

    std::vector<double> min_q(start_state.position().begin(),
                              start_state.position().end());
    std::vector<double> max_q(start_state.position().begin(),
                              start_state.position().end());
    for (const auto& state : plan_result->trajectory.state()) {
      for (int i = 0;
           i < state.position_size() && i < static_cast<int>(min_q.size());
           ++i) {
        if (state.position(i) < min_q[i]) {
          min_q[i] = state.position(i);
        }
        if (state.position(i) > max_q[i]) {
          max_q[i] = state.position(i);
        }
      }
    }

    auto format_vec = [](const auto& vals) {
      std::vector<std::string> strs;
      for (double q : vals) {
        strs.push_back(absl::StrCat(q));
      }
      return absl::StrJoin(strs, ",");
    };
    return absl::StrCat(
        "{\"ok\":true,\"code\":0,\"states\":",
        plan_result->trajectory.state_size(), ",\"start_joints\":[",
        format_vec(start_state.position()), "],\"end_joints\":[",
        format_vec(end_state.position()), "],\"min_joints\":[",
        format_vec(min_q), "],\"max_joints\":[", format_vec(max_q), "]}");
  }

  std::string PreplanMotion(absl::string_view robot_name,
                            absl::string_view b64_params) {
    if (!skill_test_factory_ || !world_client_) {
      return StatusToJson(
          absl::FailedPreconditionError("World not initialized"));
    }
    std::string raw_bytes;
    if (!absl::Base64Unescape(b64_params, &raw_bytes)) {
      return StatusToJson(absl::InvalidArgumentError("Invalid base64 payload"));
    }
    intrinsic_proto::skills::PreplanMotionParams params;
    if (!params.ParseFromString(raw_bytes)) {
      return StatusToJson(
          absl::InvalidArgumentError("Failed to parse PreplanMotionParams"));
    }

    skills::EquipmentPack equipment_pack;
    intrinsic_proto::resources::ResourceHandle handle;
    handle.set_name(std::string(robot_name));
    absl::Status add_eq_status = equipment_pack.Add("robot", handle);
    if (!add_eq_status.ok()) {
      return StatusToJson(add_eq_status);
    }

    skills::ExecuteRequest request =
        skill_test_factory_->MakeExecuteRequest(params);
    auto context = skill_test_factory_->MakeExecuteContext({
        .equipment_pack = equipment_pack,
        .world_id = std::string(kWorldId),
        .motion_planner_service = motion_planner_stub_,
        .object_world_service = object_world_service_stub_,
    });
    auto skill = skills::PreplanMotionSkill::CreateSkill();
    absl::Status status = skill->Execute(request, *context).status();
    return StatusToJson(status);
  }

 private:
  absl::Status ApplyEntityFilterAttach(
      const intrinsic_proto::skills::AttachObjectToRobotParams& params) {
    INTR_ASSIGN_OR_RETURN(world::WorldObject gripper,
                          world_client_->GetObject(params.gripper_entity()));
    INTR_ASSIGN_OR_RETURN(world::WorldObject object,
                          world_client_->GetObject(params.object_entity()));
    INTR_RETURN_IF_ERROR(world_client_->ReparentObject(
        object, gripper, GetAttachmentEntityFilter(gripper)));
    INTR_RETURN_IF_ERROR(world_client_->DisableCollisions(object, gripper));
    if (params.has_gripper_t_object()) {
      INTR_ASSIGN_OR_RETURN(Pose3d gripper_t_object,
                            FromProto(params.gripper_t_object()));
      INTR_RETURN_IF_ERROR(
          world_client_->UpdateTransform(gripper, object, gripper_t_object));
    }
    return absl::OkStatus();
  }

  std::unique_ptr<FakeWorldService> world_service_;
  std::unique_ptr<intrinsic_proto::world::internal::WorldService::Stub>
      world_service_stub_;
  std::shared_ptr<intrinsic_proto::world::ObjectWorldService::Stub>
      object_world_service_stub_;
  std::unique_ptr<world::ObjectWorldClient> world_client_;
  std::unique_ptr<MotionPlannerServiceInProcess> motion_planner_service_;
  std::shared_ptr<
      intrinsic_proto::motion_planning::v1::MotionPlannerService::StubInterface>
      motion_planner_stub_;
  std::unique_ptr<motion_planning::MotionPlannerClient> motion_planner_client_;
  std::unique_ptr<skills::SkillTestFactory> skill_test_factory_;
};

}  // namespace
}  // namespace intrinsic

int main(int argc, char** argv) {
  absl::InitializeLog();
  intrinsic::InProcessSkillRunner runner;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }
    std::vector<absl::string_view> parts = absl::StrSplit(line, '\t');
    absl::string_view cmd = parts[0];
    if (cmd == "QUIT") {
      std::cout << "{\"ok\":true,\"code\":0}" << std::endl;
      break;
    } else if (cmd == "INIT_WORLD" && parts.size() >= 3) {
      auto res = runner.InitWorld(parts[1], parts[2]);
      if (res.ok()) {
        std::cout << *res << std::endl;
      } else {
        std::cout << intrinsic::StatusToJson(res.status()) << std::endl;
      }
    } else if (cmd == "ATTACH_OBJECT" && parts.size() >= 2) {
      std::cout << runner.AttachObject(parts[1]) << std::endl;
    } else if (cmd == "DETACH_OBJECT" && parts.size() >= 2) {
      std::cout << runner.DetachObject(parts[1]) << std::endl;
    } else if (cmd == "UPDATE_WORLD" && parts.size() >= 2) {
      std::cout << runner.UpdateWorld(parts[1]) << std::endl;
    } else if (cmd == "MOVE_ROBOT" && parts.size() >= 3) {
      std::cout << runner.MoveRobot(parts[1], parts[2]) << std::endl;
    } else if (cmd == "PREPLAN_MOTION" && parts.size() >= 3) {
      std::cout << runner.PreplanMotion(parts[1], parts[2]) << std::endl;
    } else {
      std::cout << intrinsic::StatusToJson(absl::InvalidArgumentError(
                       absl::StrCat("Unknown command: ", cmd)))
                << std::endl;
    }
  }
  return 0;
}
