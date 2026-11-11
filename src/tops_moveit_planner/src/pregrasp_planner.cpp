// MoveIt 2 pregrasp planner for TOPS; EXPLICIT SERVICE TRIGGER, DRY-RUN DEFAULT.
// No compliance/controller switching is performed here. The shared supervisor
// must own controller transitions before real trajectory execution.
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class PregraspPlanner : public rclcpp::Node {
public:
  PregraspPlanner() : rclcpp::Node("tops_pregrasp_planner") {
    group_ = declare_parameter<std::string>("planning_group", "panda_arm");
    frame_ = declare_parameter<std::string>("base_frame", "panda_link0");
    ee_link_ = declare_parameter<std::string>("end_effector_link", "");
    auto topic = declare_parameter<std::string>("target_topic", "/tops/autonomy/target_pose");
    dry_run_ = declare_parameter<bool>("dry_run", true);
    allow_execute_ = declare_parameter<bool>("allow_execute", false);
    max_age_ = declare_parameter<double>("max_target_age_s", 0.5);
    pregrasp_z_ = declare_parameter<double>("pregrasp_offset_z_m", 0.12);
    min_xyz_ = declare_parameter<std::vector<double>>("workspace_min_xyz", {0.18,-0.5,0.01});
    max_xyz_ = declare_parameter<std::vector<double>>("workspace_max_xyz", {0.85,0.5,0.75});
    quaternion_ = declare_parameter<std::vector<double>>("gripper_orientation_xyzw", {1.,0.,0.,0.});
    planning_time_ = declare_parameter<double>("planning_time_s", 5.0);
    velocity_scale_ = declare_parameter<double>("velocity_scaling", 0.15);
    add_table_ = declare_parameter<bool>("add_table_collision", false);
    table_top_z_ = declare_parameter<double>("table_top_z_m", 0.0);
    table_x_ = declare_parameter<double>("table_center_x_m",0.55);
    table_y_ = declare_parameter<double>("table_center_y_m",0.0);
    table_length_ = declare_parameter<double>("table_length_m",0.9);
    table_width_ = declare_parameter<double>("table_width_m",0.8);
    table_thickness_ = declare_parameter<double>("table_thickness_m",0.06);

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic,10,[this](geometry_msgs::msg::PoseStamped::SharedPtr m){
          std::scoped_lock<std::mutex> lock(mutex_);
          last_target_ = *m;
          have_target_ = true;
        });
    service_ = create_service<std_srvs::srv::Trigger>(
      "/tops/autonomy/plan_pregrasp",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> reply){plan(*reply);});
  }

  void initialize() {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),group_);
    if (!ee_link_.empty()) move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(velocity_scale_);
    if (add_table_) {
      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id=frame_;
      table.id="tops_table";
      shape_msgs::msg::SolidPrimitive box;
      box.type=shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions={table_length_,table_width_,table_thickness_};
      geometry_msgs::msg::Pose pose;
      pose.orientation.w=1.0;
      pose.position.x=table_x_;
      pose.position.y=table_y_;
      pose.position.z=table_top_z_-table_thickness_/2.;
      table.primitives.push_back(box);
      table.primitive_poses.push_back(pose);
      table.operation=moveit_msgs::msg::CollisionObject::ADD;
      moveit::planning_interface::PlanningSceneInterface scene;
      scene.applyCollisionObject(table);
    } else {
      RCLCPP_WARN(get_logger(),"Table collision geometry OFF: configure measured scene before execution");
    }
    RCLCPP_WARN(get_logger(),"dry_run=%s allow_execute=%s; use explicit Trigger service",
                dry_run_?"true":"false",allow_execute_?"true":"false");
  }

private:
  void plan(std_srvs::srv::Trigger::Response& reply) {
    geometry_msgs::msg::PoseStamped target;
    {
      std::scoped_lock<std::mutex> lock(mutex_);
      if (!have_target_) { reply.message="No target received"; return; }
      target=last_target_;
    }
    if (target.header.frame_id!=frame_) {
      reply.message="Wrong target coordinate frame; explicit TF conversion required";return;
    }
    rclcpp::Time t(target.header.stamp);
    if (t.nanoseconds()==0) {reply.message="Target timestamp missing"; return;}
    double age=(now()-t).seconds();
    if (age < 0 || age > max_age_) {
      reply.message="Target too old or in future";return;
    }
    if (min_xyz_.size()!=3||max_xyz_.size()!=3||quaternion_.size()!=4) {
      reply.message="Bad workspace or orientation vector parameter lengths";return;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id=frame_;
    goal.header.stamp=now();
    goal.pose.position=target.pose.position;
    goal.pose.position.z+=pregrasp_z_;
    const double coords[3]={goal.pose.position.x,goal.pose.position.y,goal.pose.position.z};
    for(int i=0;i<3;i++) {
      if (!std::isfinite(coords[i]) || coords[i]<min_xyz_[i] || coords[i]>max_xyz_[i]) {
        reply.message="Pregrasp outside configured workspace";return;
      }
    }
    double qnorm=0.; for(double q:quaternion_) qnorm+=q*q;
    if (std::fabs(qnorm-1.0)>0.01) {reply.message="Configure normalized gripper orientation";return;}
    goal.pose.orientation.x=quaternion_[0];
    goal.pose.orientation.y=quaternion_[1];
    goal.pose.orientation.z=quaternion_[2];
    goal.pose.orientation.w=quaternion_[3];

    move_group_->setPoseTarget(goal);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto status=move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (status!=moveit::core::MoveItErrorCode::SUCCESS) {
      reply.message="MoveIt failed to plan collision-free pregrasp";return;
    }
    reply.success=true;
    reply.message="Pregrasp plan found; not executed";
    if (dry_run_||!allow_execute_) return;
    // WARNING: caller must ensure supervisory state PLAN and controller ownership.
    const auto exec=move_group_->execute(plan);
    reply.success=(exec==moveit::core::MoveItErrorCode::SUCCESS);
    reply.message=reply.success?"Executed pregrasp trajectory":"MoveIt execution failed";
  }

  std::string group_,frame_,ee_link_;
  bool dry_run_{true},allow_execute_{false},have_target_{false},add_table_{false};
  double max_age_,pregrasp_z_,planning_time_,velocity_scale_;
  double table_top_z_,table_x_,table_y_,table_length_,table_width_,table_thickness_;
  std::vector<double> min_xyz_,max_xyz_,quaternion_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_target_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<PregraspPlanner>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec](){exec.spin();});
  try { node->initialize(); }
  catch(const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(),"MoveIt initialization failed: %s",e.what());
    exec.cancel(); spin_thread.join();rclcpp::shutdown();return 1;
  }
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
