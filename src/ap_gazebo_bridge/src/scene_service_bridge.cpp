// Persistent scene services missing from the Garden build of ros_gz_bridge 0.244.
#include <memory>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>

class SceneServiceBridge : public rclcpp::Node
{
  using SetPose = ros_gz_interfaces::srv::SetEntityPose;
  using Spawn = ros_gz_interfaces::srv::SpawnEntity;
  using Delete = ros_gz_interfaces::srv::DeleteEntity;

public:
  SceneServiceBridge() : Node("scene_service_bridge")
  {
    const auto prefix = "/world/" + declare_parameter<std::string>("world_name", "competition_arena");
    // Single executor, short bounded requests and one in-flight request per Python actor.
    // The same transport node/connections are retained throughout the mission.
    pose_ = create_service<SetPose>(prefix + "/set_pose",
      [this, prefix](const SetPose::Request::SharedPtr request, SetPose::Response::SharedPtr response) {
        gz::msgs::Pose pose;
        pose.set_name(request->entity.name);
        pose.set_id(request->entity.id);
        copy_pose(request->pose, pose);
        response->success = send(prefix + "/set_pose", pose);
      });
    spawn_ = create_service<Spawn>(prefix + "/create",
      [this, prefix](const Spawn::Request::SharedPtr request, Spawn::Response::SharedPtr response) {
        const auto & factory = request->entity_factory;
        gz::msgs::EntityFactory message;
        message.set_name(factory.name);
        message.set_allow_renaming(factory.allow_renaming);
        // Gazebo chooses among these by field presence, including empty values.
        if (!factory.sdf.empty()) {
          message.set_sdf(factory.sdf);
        } else if (!factory.sdf_filename.empty()) {
          message.set_sdf_filename(factory.sdf_filename);
        } else if (!factory.clone_name.empty()) {
          message.set_clone_name(factory.clone_name);
        } else {
          response->success = false;
          return;
        }
        message.set_relative_to(factory.relative_to);
        copy_pose(factory.pose, *message.mutable_pose());
        response->success = send(prefix + "/create", message);
      });
    delete_ = create_service<Delete>(prefix + "/remove",
      [this, prefix](const Delete::Request::SharedPtr request, Delete::Response::SharedPtr response) {
        gz::msgs::Entity message;
        message.set_name(request->entity.name);
        message.set_id(request->entity.id);
        message.set_type(static_cast<gz::msgs::Entity_Type>(request->entity.type));
        response->success = send(prefix + "/remove", message);
      });
    RCLCPP_INFO(get_logger(), "Persistent Gazebo scene services ready: %s", prefix.c_str());
  }

private:
  static void copy_pose(const geometry_msgs::msg::Pose & source, gz::msgs::Pose & target)
  {
    target.mutable_position()->set_x(source.position.x);
    target.mutable_position()->set_y(source.position.y);
    target.mutable_position()->set_z(source.position.z);
    target.mutable_orientation()->set_x(source.orientation.x);
    target.mutable_orientation()->set_y(source.orientation.y);
    target.mutable_orientation()->set_z(source.orientation.z);
    target.mutable_orientation()->set_w(source.orientation.w);
  }

  template<typename Request>
  bool send(const std::string & service, const Request & request)
  {
    gz::msgs::Boolean reply;
    bool result = false;
    const bool received = transport_.Request(service, request, 200, reply, result);
    if (!received || !result || !reply.data()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Gazebo service unavailable or refused request: %s", service.c_str());
      return false;
    }
    return true;
  }

  gz::transport::Node transport_;
  rclcpp::Service<SetPose>::SharedPtr pose_;
  rclcpp::Service<Spawn>::SharedPtr spawn_;
  rclcpp::Service<Delete>::SharedPtr delete_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneServiceBridge>());
  rclcpp::shutdown();
  return 0;
}
