#include <homodeus_arm_interface/ArmInterfaceNode.h>

ArmInterfaceNode::ArmInterfaceNode(ros::NodeHandle n):
nh{n}, 
gac("/gripper_controller/follow_joint_trajectory", true),
tac("/torso_controller/follow_joint_trajectory", true),
hac("/head_controller/follow_joint_trajectory", true)
{
    ROS_INFO("Node init strated");

    std::cout << "2asdddddddd" << std::endl;
    pick_pose_sub = nh.subscribe("/object_detector/object_pose", 5, &ArmInterfaceNode::pickPoseCB, this);
    drop_pose_sub = nh.subscribe("/drop_point", 5, &ArmInterfaceNode::dropPoseCB, this);
    close_gripper_goal.trajectory = closedGripper();
    open_gripper_goal.trajectory = openedGripper();

    close_schunk_gripper_goal.trajectory = closedSchunkGripper();
    open_schunk_gripper_goal.trajectory = openedSchunkGripper();
    
    go_up.trajectory = goUp();
    look_down.trajectory = lookDown();
    
    ROS_INFO("Waiting for gripper joint controller server...");
    std::cout << "Waiting for gripper joint controller server..." << std::endl;
    //print("Waiting for gripper joint controller server...")
    gac.waitForServer();
    ROS_INFO("Found  gripper joint controller server");
    
    ROS_INFO("Waiting for torso joint controller server...");
    tac.waitForServer();
    ROS_INFO("Found  torso joint controller server");
    
    ROS_INFO("Waiting for head joint controller server...");
    hac.waitForServer();
    ROS_INFO("Found  head joint controller server");
    
    // HBBA observer topics
    bhvr_output_pick_result = nh.advertise<std_msgs::Bool>("/bhvr_output_pick_result", 5);
    bhvr_output_place_result = nh.advertise<std_msgs::Bool>("/bhvr_output_place_result", 5);

    ROS_INFO("Node init done");
}

// UNUSED: Use these instead of the gripper ones for Hey5 hand
trajectory_msgs::JointTrajectory closedFingers()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(3);
    close_fingers.joint_names[0] = "hand_index_joint";
    close_fingers.joint_names[1] = "hand_mrl_joint";
    close_fingers.joint_names[2] = "hand_thumb_joint"; 
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(3);
    close_fingers.points[0].positions[0] = 0.80;
    close_fingers.points[0].positions[1] = 1.60;
    close_fingers.points[0].positions[2] = 0.85;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::openedFingers()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(3);
    close_fingers.joint_names[0] = "hand_index_joint";
    close_fingers.joint_names[1] = "hand_mrl_joint";
    close_fingers.joint_names[2] = "hand_thumb_joint"; 
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(3);
    close_fingers.points[0].positions[0] = 0;
    close_fingers.points[0].positions[1] = 0;
    close_fingers.points[0].positions[2] = 0;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::closedGripper()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(2);
    close_fingers.joint_names[0] = "gripper_left_finger_joint";
    close_fingers.joint_names[1] = "gripper_right_finger_joint";
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(2);
    close_fingers.points[0].positions[0] = 0.01;
    close_fingers.points[0].positions[1] = 0.01;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::openedGripper()
{
    trajectory_msgs::JointTrajectory open_fingers;
    open_fingers.joint_names.resize(2);
    open_fingers.joint_names[0] = "gripper_left_finger_joint";
    open_fingers.joint_names[1] = "gripper_right_finger_joint";
    open_fingers.points.resize(1);
    open_fingers.points[0].positions.resize(2);
    open_fingers.points[0].positions[0] = 0.045;
    open_fingers.points[0].positions[1] = 0.045;
    open_fingers.points[0].time_from_start = ros::Duration(0.5);
    return open_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::closedSchunkGripper()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(1);
    close_fingers.joint_names[0] = "gripper_finger_joint";
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(1);
    close_fingers.points[0].positions[0] = 0.01;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::openedSchunkGripper()
{
    trajectory_msgs::JointTrajectory open_fingers;
    open_fingers.joint_names.resize(1);
    open_fingers.joint_names[0] = "gripper_finger_joint";
    open_fingers.points.resize(1);
    open_fingers.points[0].positions.resize(1);
    open_fingers.points[0].positions[0] = 0.030;
    open_fingers.points[0].time_from_start = ros::Duration(0.5);
    return open_fingers;
}
trajectory_msgs::JointTrajectory ArmInterfaceNode::goUp()
{
    trajectory_msgs::JointTrajectory go_up;
    go_up.joint_names.resize(1);
    go_up.joint_names[0] = "torso_lift_joint";
    go_up.points.resize(1);
    go_up.points[0].positions.resize(1);
    go_up.points[0].positions[0] = 0.3;
    go_up.points[0].time_from_start = ros::Duration(3);
    return go_up;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::lookDown()
{
    trajectory_msgs::JointTrajectory look_down;
    look_down.joint_names.resize(2);
    look_down.joint_names[0] = "head_1_joint";
    look_down.joint_names[1] = "head_2_joint";
    look_down.points.resize(1);
    look_down.points[0].positions.resize(2);
    look_down.points[0].positions[0] = 0;
    look_down.points[0].positions[1] = -0.5;
    look_down.points[0].time_from_start = ros::Duration(3);
    return look_down;
}

void ArmInterfaceNode::gotoInitPose()
{

    ROS_INFO("Going up to look position");
    tac.sendGoalAndWait(go_up, ros::Duration(3));
    ROS_INFO("Looking down");
    hac.sendGoalAndWait(look_down, ros::Duration(3));
    ROS_INFO("Ready");

}

void ArmInterfaceNode::pickPoseCB(const geometry_msgs::PoseStampedConstPtr posestamped)
{
    pick_point = *posestamped;
    got_pick_pose = true;
    bool success = false;

    ROS_INFO("Going to grasp preparation pose");
    success = gotoGraspPrep();
    if (success)
    {
        ROS_INFO("Now at grasp preparation pose, opening gripper");
        gac.sendGoalAndWait(open_gripper_goal, ros::Duration(2));
        //gac.sendGoalAndWait(open_schunk_gripper_goal, ros::Duration(2));
    }
    else
    {
        ROS_INFO("Failed to go to grasp preparation pose in time, will still attempt rest of pick sequence");
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(posestamped->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = posestamped->pose.position.x;
    auto y  = posestamped->pose.position.y;
    auto z  = posestamped->pose.position.z;
    std::cout << "Pos : " << x << ", " << y <<", " << z << std::endl;
    std::cout << "roll,pithc,yaw : " << roll << ", " << pitch <<", " << yaw << std::endl;
    ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    success = moveToCartesian(x-0.4, y, z-0.05, roll+1.571, pitch, yaw);
    ros::Duration(1).sleep();
    if (success)
    {
        ROS_INFO("arm_interface_node: reached first wayp1oint");
        success = moveToCartesian(x-0.15, y, z-0.05, roll+1.571, pitch, yaw);
    }

    if (success)
    {
        ROS_INFO("arm_interface_node: successfully moved to pick point, closing gripper...");
        gac.sendGoalAndWait(close_gripper_goal, ros::Duration(2));
        //gac.sendGoalAndWait(close_schunk_gripper_goal, ros::Duration(2));
        ROS_INFO("Closed!");
        //
        ROS_INFO("Going up!");
        moveToCartesian(x-0.15, y, z+0.5, roll+1.571, pitch, yaw);

        ROS_INFO("Retreating");
        success = gotoRetreat(pick_point);
    }
    else
        ROS_INFO("arm_interface_node: failed to go to pick point!");

    
    if (success)
    {
        ROS_INFO("arm_interface_node: successfully retreated from pick point.");
        ROS_INFO("Going to carrying pose");
        success  = goHome();
    }
    else
        ROS_INFO("arm_interface_node: failed to retreat from pick point!");

    std_msgs::Bool pick_success_msg;
    pick_success_msg.data = success;
    bhvr_output_pick_result.publish(pick_success_msg);
}

void ArmInterfaceNode::dropPoseCB(const geometry_msgs::PoseStampedConstPtr posestamped)
{
    drop_point = *posestamped;
    got_drop_pose = true;
    bool success = true;

    ROS_INFO("Going to drop preparation pose");
    success = gotoGraspPrep();
    // success = moveToJoint(0.35, 0.15, 0.00, -1.08, 2.29, 0.33, 0.27, -2.07);
    if (success)
    {
        ROS_INFO("Now at drop preparation pose");
    }
    else
    {
        ROS_ERROR("Failed to go to drop preparation pose in time, will still attempt rest of pick sequence");
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(posestamped->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = posestamped->pose.position.x;
    auto y  = posestamped->pose.position.y;
    auto z  = posestamped->pose.position.z;
    
    ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    success = moveToCartesian(x-0.2, y, z+0.05, roll, pitch, yaw);
    if (success)
    {
        ROS_INFO("arm_interface_node: reached first waypoint");
        success = moveToCartesian(x, y, z, roll, pitch, yaw);
    }

    if (success)
    {
        ROS_INFO("arm_interface_node: successfully moved to drop point, opening gripper...");
        gac.sendGoalAndWait(open_gripper_goal, ros::Duration(2));
        ROS_INFO("Opened!");
    }
    else
        ROS_INFO("arm_interface_node: failed to go to drop point!");

    if(success)
    {
        success = gotoRetreat(drop_point);
    }
    
    if (success)
    {
        ROS_INFO("arm_interface_node: successfully retreated from drop point.");
        ROS_INFO("Going home");
        success = goHome();
    }
    else
        ROS_INFO("arm_interface_node: failed to retreat from drop point!");

    std_msgs::Bool drop_success_msg;
    drop_success_msg.data = success; 
    bhvr_output_place_result.publish(drop_success_msg);
    
}

bool ArmInterfaceNode::goHome()
{
    bool success;
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, 0.01, 2.10, -1.5, 1.37, 0.0);
    success = moveToJoint(0.25, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0);
    return success;
}

bool ArmInterfaceNode::gotoCarryPose()
{
    bool success;
    // success = moveToJoint(0.30, 0.10, 0.00, -1.72, 2.21, 0.00, 0.05, 0.00);
    success = moveToJoint(0.35, 0.15, 0.00, -1.08, 2.29, 0.33, 0.27, -2.07);

    ros::Duration(1).sleep();

    success = moveToJoint(0.20, 0.20, 0.0, 0.0,  2.18, -1.17, 1.01, -1.78);
    return success;
}

bool ArmInterfaceNode::gotoGraspPrep()
{
    bool success;
    success = moveToJoint(0.34, 0.20, 0.79, 0.01, 2.10, -1.5, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
    return success;
}

bool ArmInterfaceNode::gotoRetreat(const geometry_msgs::PoseStamped posestamped)
{
    ROS_INFO("Attempting retreat");

    tf::Quaternion quat;
    tf::quaternionMsgToTF(posestamped.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = posestamped.pose.position.x;
    auto y  = posestamped.pose.position.y;
    auto z  = posestamped.pose.position.z;
    return moveToCartesian(x-0.1, y, z+0.2, roll, pitch, yaw);

}

// Code to use the arm interface
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~"); 
    std::cout << "asdddddddd" << std::endl;
    
    ArmInterfaceNode arm_node(n);
    std::cout << "asdddddddd" << std::endl;

    arm_node.gotoInitPose();
    

    ros::AsyncSpinner spinner(1);
    spinner.start();

    double frequency = 5;
    ros::Rate rate(frequency);
    while ( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}
