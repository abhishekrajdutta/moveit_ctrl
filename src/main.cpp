#include<ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;


void moveitnode(std::vector<geometry_msgs::Pose> wpts)
{
	static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

   namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
 
 

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  

   move_group.setStartStateToCurrentState();

  wpts.push_back(move_group.getCurrentPose().pose);
  moveit_msgs::RobotTrajectory trajectory_msg;
  sleep(3.0);
  move_group.setPlanningTime(20.0);
  
  double fraction = move_group.computeCartesianPath(wpts,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);

  my_plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);    
  sleep(5.0);
  
  move_group.execute(my_plan); 
}


class Goals
{
public:
  Goals();


private:
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void setpose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  ros::NodeHandle node_handle;
  ros::Subscriber pose_sub;
  int count=0;
  std::vector<geometry_msgs::Pose> wpts;
  geometry_msgs::Pose start;

};


Goals::Goals()
{
  ros::NodeHandle node_handle;  
  pose_sub = node_handle.subscribe<geometry_msgs::PoseStamped>("/aruco_goal",10,&Goals::goalCallback,this);
}


void Goals::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	cout<<"recieved message!!!"<<endl;
  cout<<msg->pose.position.x<<endl;
	setpose(msg);
}

void Goals::setpose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::Pose p;
  // p.orientation.w = 1.0;
  p=msg->pose;
  // p.position.z = 1.1 + p.position.z ;
  // p.position.x = 0.7 + p.position.x ;
    if (count<3)
    {
        cout<<"adding!!"<<endl;
        wpts.push_back(p);
	      count+=1;
    }
    else if (count==3)
    {
    	cout<<"ready!!"<<endl;
      count+=1;
      cout<<"moveit!!"<<endl;
      count=0;
      moveitnode(wpts);
    }
    
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Goals goal;
  // moveitnode();
   

  ros::waitForShutdown();
  return 0;
}


  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  // robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_gp");

  // Second get a RobotTrajectory from trajectory
  // rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
  
  // Thrid create a IterativeParabolicTimeParameterization object
  // trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  // bool success = iptp.computeTimeStamps(rt);
  // ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  // rt.getRobotTrajectoryMsg(trajectory_msg);
  // Check trajectory_msg for velocities not empty
  // std::cout << trajectory_msg << std::endl;