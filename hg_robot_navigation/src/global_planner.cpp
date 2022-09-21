
#include <angles/angles.h>
#include <hg_robot_navigation/global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "global_planner.h"
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner) //插件形式导入，继承baseglobalplanner包 ，自定义global——planner

namespace global_planner
{
  GlobalPlanner::GlobalPlanner()
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){}

  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  GlobalPlanner::~GlobalPlanner() {
    // deleting a nullptr is a noop
    delete world_model_;
  }
/**
 * @Date: 2022-08-08 23:37:41
 * @Input :  ,Return:
 * @Functonname :
 * @MethodAuthor: Rolle,robot_luoer@163.com
 * @Description :  
 * @功能 :name 名称  costmap_ros 地图信息。
*/
void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }
  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double GlobalPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }
/**
 * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
 * @param x_i The x position of the robot 
 * @param y_i The y position of the robot 
 * @param theta_i The orientation of the robot
 * @return 
 **/

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }


    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 1.0;
    double dScale = 0.01;

    while(!done)
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = start_yaw;
        ROS_WARN("The carrot planner could not find a valid plan for this goal");
        break;
      }
      target_x = start_x + scale * diff_x;
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
      
      double footprint_cost = footprintCost(target_x, target_y, target_yaw);
      if(footprint_cost >= 0)
      {
          done = true;
      }
      scale -=dScale;
    }

    plan.push_back(start);
    geometry_msgs::PoseStamped new_goal = goal;
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0, 0, target_yaw);

    new_goal.pose.position.x = target_x;
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    return (done);
  }

};
