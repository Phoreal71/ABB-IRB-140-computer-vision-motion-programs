// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

//launch with roslaunch irb140_description irb140.launch, which places a block at x=0.5, y=0
// launch action server: rosrun irb140_planner irb140_cart_move_as
// then run this node

//for manual gripper control,  rosrun cwru_sticky_fingers finger_control_dummy_node /sticky_finger/link6 false

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <std_srvs/SetBool.h>
using namespace std;

double toy_pose_x;
double toy_pose_y;
double toy_pose_z;

int mark = 0;
int mark2 = 0;

void callBack(const geometry_msgs::PoseStamped& msg){
  if(mark == 0){  
	toy_pose_x = msg.pose.position.x;
	toy_pose_y = msg.pose.position.y;
	toy_pose_z = msg.pose.position.z;
	mark++;
  }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "Move_140_to_pickup_toy"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
     //Subscriber to openCV node
    ros::Subscriber sub = nh.subscribe("object_pose",1,callBack);
    std_srvs::SetBool srv;
    srv.request.data = true;

    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;

    nsteps = 10;
    arrival_time = 2.0;



    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;

    while (ros::ok()) {

	//Move robot to gripper down position first before anything with translation of 0.450, -0.000, 0.367
        b_des << 0, 0, -1;
        n_des << -1, 0, 0;
        t_des = b_des.cross(n_des);

        R_gripper.col(0) = n_des;
        R_gripper.col(1) = t_des;
        R_gripper.col(2) = b_des;

        O_des << 0.45,0.3,0.367;
        Eigen::Affine3d tool_affine;
        tool_affine.linear() = R_gripper;
        tool_affine.translation() = O_des;
        tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
        ROS_INFO("requesting plan to gripper-down pose:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time,     tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }


	//Get kong dog toy centroid coordinates from opencv node
	ros::spinOnce();

        //move to kong dog toy
        ROS_INFO("moving to above dog toy");
        tool_pose.pose.position.x=toy_pose_x; 
	tool_pose.pose.position.y=toy_pose_y; 
	tool_pose.pose.position.z = 0.35; 
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        } 

	//Turn vacuum gripper on 
	system("sudo usbrelay BITFT_1=1");        

	//Move robot arm down to drop off toy
	ROS_INFO("Descending to dog toy");
	tool_pose.pose.position.z = toy_pose_z; 
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }           

	//Ascend until z = 0.35
        ROS_INFO("Ascending with kong toy");
        tool_pose.pose.position.z = 0.35;           
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	//Move to (0.75, 0.0, 0.35) -- hardcoded dropoff location
	ROS_INFO("Moving to drop off location");
        tool_pose.pose.position.x = 0.75;
	tool_pose.pose.position.y = 0.0;      
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	//Descend until dog toy reaches dropoff location
	ROS_INFO("Descending");
        tool_pose.pose.position.z = 0.125;
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	//Turn vacuum gripper off 
	system("sudo usbrelay BITFT_1=0");

	//Ascend to z = 0.35
	ROS_INFO("Ascending");
        tool_pose.pose.position.z = 0.35;
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	// Ensures robot picks dog toy up only once (temporary)
	mark2++;
	if(mark2 != 0){
	  return 0;
	}

	/* Commented out section moves robot to a roughly home position
	//Move tool flange back to home position first
	b_des << 1,0,0;
        n_des << 0,0,-1;
        t_des << 0,1,0;

    	R_gripper.col(0) = n_des;
    	R_gripper.col(1) = t_des;
    	R_gripper.col(2) = b_des;

    	//- Translation: [0.450, -0.000, 0.367]

    	O_des << 0.515000 + 0.20, 0.0, 0.711999;
    	Eigen::Affine3d tool_affine;
    	tool_affine.linear() = R_gripper;
    	tool_affine.translation() = O_des;
    	//   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    	tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
	//Move to home position 
	 ROS_INFO("Moving to home position");      
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
	*/
	
    }
	      
    return 0;
}

