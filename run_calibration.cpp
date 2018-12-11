//Program used to calibrate ABB 140 robot by collecting 10 camera pixel and robot coordinate sets
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "Calibration_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
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
	
    //Variable used for user input
    int input;

    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << -1, 0, 0;
    t_des = b_des.cross(n_des);

    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;

    //- Translation: [0.450, -0.000, 0.367]

    O_des << 0.45,0.3,0.367;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation() = O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    while (ros::ok()) {
	
	cout << "Enter 1 to move robot to first calibration point of (0.2, 0.63, 0.018)" << endl;
	cin >> input;
 	if(input == 1){
          ROS_INFO("Moving to point 1");
          tool_pose.pose.position.x=0.21;
	  tool_pose.pose.position.y=0.63;
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	// 0.515000 + 0.20, 0.0, 0.711999
	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter 2 to move robot to second calibration point of (0.2, 0.7, 0.018)" << endl;
	cin >> input;
 	if(input == 2){
          ROS_INFO("Moving to point 2");
          tool_pose.pose.position.x=0.2; 
	  tool_pose.pose.position.y=0.7; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}
     
	cout << "Enter 3 to move robot to third calibration point of (0.08, 0.73, 0.018)" << endl;
	cin >> input;
 	if(input == 3){
          ROS_INFO("Moving to point 3");
          tool_pose.pose.position.x=0.08; 
	  tool_pose.pose.position.y=0.73; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter 4 to move robot to fourth calibration point of (0.2, 0.55, 0.018)" << endl;
	cin >> input;
 	if(input == 4){
          ROS_INFO("Moving to point 4");
          tool_pose.pose.position.x=0.2; 
	  tool_pose.pose.position.y=0.55; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	
	cout << "Enter 5 to move robot to fifth calibration point of (0.1, 0.53, 0.018)" << endl;
	cin >> input;
 	if(input == 5){
          ROS_INFO("Moving to point 5");
          tool_pose.pose.position.x=0.1; 
	  tool_pose.pose.position.y=0.53; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	
	cout << "Enter 6 to move robot to sixth calibration point of (0.42, 0.60, 0.018)" << endl;
	cin >> input;
 	if(input == 6){
          ROS_INFO("Moving to point 6");
          tool_pose.pose.position.x=0.42; 
	  tool_pose.pose.position.y=0.60; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter 7 to move robot to seventh calibration point of (0.42, 0.64, 0.018)" << endl;
	cin >> input;
 	if(input == 7){
          ROS_INFO("Moving to point 6");
          tool_pose.pose.position.x=0.42; 
	  tool_pose.pose.position.y=0.64; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter 8 to move robot to eighth calibration point of (0.42, 0.55, 0.018)" << endl;
	cin >> input;
 	if(input == 8){
          ROS_INFO("Moving to point 8");
          tool_pose.pose.position.x=0.42; 
	  tool_pose.pose.position.y=0.55; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter 9 to move robot to nineth calibration point of (0.1, 0.6, 0.018)" << endl;
	cin >> input;
 	if(input == 9){
          ROS_INFO("Moving to point 9");
          tool_pose.pose.position.x=0.1; 
	  tool_pose.pose.position.y=0.6; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter 10 to move robot to tenth calibration point of (0.1, 0.56, 0.018)" << endl;
	cin >> input;
 	if(input == 10){
          ROS_INFO("Moving to point 10");
          tool_pose.pose.position.x=0.10; 
	  tool_pose.pose.position.y=0.56; 
	  tool_pose.pose.position.z =0.018; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	cout << "Enter '0' to move robot out of camera view" << endl;
	cin >> input;
	if(input == 0){
	  ROS_INFO("Moving out of camera view");
          tool_pose.pose.position.x=0.5;
	  tool_pose.pose.position.y=0.4;
	  tool_pose.pose.position.z =0.35; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
              ROS_INFO("successful plan; command execution of trajectory");
              rtn_val = cart_motion_commander.execute_planned_traj();
              ros::Duration(arrival_time + 0.2).sleep();
          } else {
              ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }      
	}

	return 0;
	
    }
	      
}

