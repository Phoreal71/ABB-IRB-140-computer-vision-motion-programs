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

const double DROPOFF_HEIGHT=0.125;
const double PICKUP_HEIGHT= 0.0145; //this value comes  from object finder publication
const double DROPOFF_X = 0.75;
const double DROPOFF_Y = 0.0;

const double APPROACH_CLEARANCE = 0.12; //approach/depart height above actual grasp or dropoff pose
const double ARRIVAL_TIME = 0.2; //tune this. 1 second move time!!
const double SLOW_ARRIVAL_TIME = 1.25; //1.5;


double toy_pose_x;
double toy_pose_y;
double toy_pose_z;

geometry_msgs::Quaternion toy_orientation;

int mark = 0;
int mark2 = 0;
bool got_snapshot = false;
int ans;

void callBack(const geometry_msgs::PoseStamped& msg){
  if(!got_snapshot){  
	toy_pose_x = msg.pose.position.x;
	toy_pose_y = msg.pose.position.y;
	toy_pose_z = msg.pose.position.z;
	toy_orientation = msg.pose.orientation;
	got_snapshot=true;
  }
}

/*
double XformUtils::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

geometry_msgs::Quaternion  XformUtils::convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}
*/

//given quaternion for orientation of object, compute the corresponding R matrix for the TOOL
//tool z-axis constrained to point DOWN, and tool x-axis aligned w/ object x-axis
Eigen::Matrix3d tool_R_from_object_orientation(geometry_msgs::Quaternion object_orientation) {

    double quat_z = object_orientation.z;
    double quat_w = object_orientation.w;
    double phi = 2.0 * atan2(quat_z, quat_w); //this is rotation about z, pointing UP
    //get corresponding direction of x axis:
    ROS_INFO("object angle = %f",phi);
    Eigen::Vector3d x_vec,y_vec,z_vec;
    x_vec<<cos(phi),sin(phi),0.0;
    z_vec<<0,0,-1;
    y_vec = z_vec.cross(x_vec);
    Eigen::Matrix3d tool_R;
    tool_R.col(0)= x_vec;
    tool_R.col(1)= y_vec;
    tool_R.col(2)= z_vec;
    return tool_R;

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
    //double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;

    Eigen::Matrix3d R_gripper,R_tool_dropoff,R_tool_pickup;//R_tool_wrt_object;
    Eigen::Vector3d b_des,n_des,t_des,O_des;

    //Turn vacuum gripper off 
    system("sudo usbrelay BITFT_1=0");

    nsteps = 10;


    //arrival_time = 1.0; //2.0; //wsn: 12/12/18, try reducing move time
        b_des << 0, 0, -1;
        n_des << -1, 0, 0;
        t_des = b_des.cross(n_des);

        R_gripper.col(0) = n_des;
        R_gripper.col(1) = t_des;
        R_gripper.col(2) = b_des;
	R_tool_dropoff = R_gripper; //defaults
        R_tool_pickup = R_gripper;

        Eigen::Affine3d tool_affine;
	//Move robot to gripper down position first before anything with translation of 0.450, -0.000, 0.367

        //O_des << 0.45,0.3,0.367;
        //wsn: change initial pose height: 12/12/18
        O_des << 0.45,0.3,PICKUP_HEIGHT+APPROACH_CLEARANCE;

        tool_affine.linear() = R_tool_dropoff;
        tool_affine.translation() = O_des;
        tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");

        cout<<"enter 1 for robot to move to pre-pose: ";
        cin>>ans;

        ROS_INFO("requesting plan to gripper-down pose:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, ARRIVAL_TIME,     tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            //ros::Duration(ARRIVAL_TIME + 0.5).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

   bool go_on=true;
    while (ros::ok()) {
      //Turn vacuum gripper off 
      system("sudo usbrelay BITFT_1=0");

        cout<<"enter 1 to pick up object; 0 to quit: ";
        cin>>ans;
        if (ans==0) { //return robot to its home position and quit
         //int plan_jspace_traj_current_to_qgoal(int nsteps, double arrival_time,Eigen::VectorXd q_goal);
          Eigen::VectorXd q_goal;
          q_goal.resize(6);
          q_goal<<0,0,0,0,0,0;
          ROS_INFO_STREAM("sending to q_goal = "<<q_goal.transpose()<<endl);
          rtn_val = cart_motion_commander.plan_jspace_traj_current_to_qgoal(nsteps,3.0,q_goal);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
          } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
          }
         //terminate this node;
         return 0;
        }

        //if here, proceed with manipulation:

	//Get kong dog toy centroid coordinates from opencv node
	got_snapshot=false;
        ROS_INFO("waiting for object pose from camera node...");
        //keep testing until have  a valid image and coords
        while (!got_snapshot) {
	   ros::spinOnce();
           ros::Duration(0.5).sleep();
        }
        go_on=true;
        //cout<<"enter 1 to continue: ";
        //cin>>ans;
        //move to kong dog toy
        ROS_INFO("moving to above dog toy");

        //specify pickup pose as an affine, then convert to a geometry_msgs/Pose
        O_des << toy_pose_x,toy_pose_y,PICKUP_HEIGHT+APPROACH_CLEARANCE;
        //change R_pickup to align gripper x-axis with object x-axis:
        R_tool_pickup= tool_R_from_object_orientation(toy_orientation);
        tool_affine.linear() = R_tool_pickup;
        tool_affine.translation() = O_des;
        tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
        xformUtils.printPose(tool_pose);

        //adjust arrival time for speed of move
        rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, ARRIVAL_TIME, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            go_on = false;
        } 
        //cout<<"enter 1 to continue: ";
        //cin>>ans;
	//Turn vacuum gripper on 
        if(go_on)
	   system("sudo usbrelay BITFT_1=1");        

	//Move robot arm down to pick up toy
        if(go_on) {
           //cout<<"enter 1 to continue: ";
           //cin>>ans;
	  ROS_INFO("Descending to dog toy");
	  tool_pose.pose.position.z = toy_pose_z; 
          xformUtils.printPose(tool_pose);
          rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, ARRIVAL_TIME, tool_pose);
          if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
          } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            go_on=false;
          } 
        }          

      if(go_on) {
        //cout<<"enter 1 to continue: ";
        //cin>>ans;
	//Ascend until z = 0.35
        ROS_INFO("Ascending with kong toy");
        tool_pose.pose.position.z = toy_pose_z+APPROACH_CLEARANCE; //0.35;           
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, ARRIVAL_TIME, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            go_on=false;
        }
       }

       if (go_on) {
        //cout<<"enter 1 to continue: ";
        //cin>>ans;
	//Move to (0.75, 0.0, 0.35) -- hardcoded dropoff location
	ROS_INFO("Moving to drop off approach location");

        O_des << DROPOFF_X,DROPOFF_Y,DROPOFF_HEIGHT+APPROACH_CLEARANCE;

        tool_affine.linear() = R_tool_dropoff; //align x axis with robot x axis
        tool_affine.translation() = O_des;

        tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
        xformUtils.printPose(tool_pose);

        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        //rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, SLOW_ARRIVAL_TIME, tool_pose);
        rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, SLOW_ARRIVAL_TIME, tool_pose);

        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            go_on=false;
        }
      }

     if (go_on) {
        //cout<<"ready to plan/execute descend move: enter 1 to continue: ";
        //cin>>ans;
	//Descend until dog toy reaches dropoff location
	ROS_INFO("Descending");
        tool_pose.pose.position.z = DROPOFF_HEIGHT; //0.125;
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, ARRIVAL_TIME, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            go_on=false;
        }
      }

	//Turn vacuum gripper off 
	system("sudo usbrelay BITFT_1=0");

	//Ascend to after release:
      if(go_on) {
        //cout<<"enter 1 to plan/execute depart move:  ";
        //cin>>ans;
	ROS_INFO("Ascending");
        tool_pose.pose.position.z = DROPOFF_HEIGHT+APPROACH_CLEARANCE; //0.35;
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_current_to_des_tool_pose(nsteps, ARRIVAL_TIME, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
            go_on=false;
        }
       }
     
        //get ready for another iteration; return to pre-pose over  (3) seconds

        ROS_INFO("returning to pre-pose...");
        O_des << 0.45,0.3,PICKUP_HEIGHT+APPROACH_CLEARANCE;

        tool_affine.linear() = R_tool_dropoff;
        tool_affine.translation() = O_des;
        tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
        ROS_INFO("requesting plan to gripper-down pose:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, 1.0, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(0.5).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
	
    }
	      
    return 0;
}

