#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include "ur5_cpp_controller/GoToPose.h" 

class UR5Move {
public:
    UR5Move(ros::NodeHandle& nh);
    void control();

private:
    void moveToPose();
    bool goToPoseGoalCallback(ur5_cpp_controller::GoToPose::Request& req, ur5_cpp_controller::GoToPose::Response& res); // Updated to use the GoToPose service

    bool openGripperCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool closeGripperCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void openGripper();
    void closeGripper();

    ros::NodeHandle nodeHandle_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    ros::ServiceClient dynamixel_client_;
    ros::ServiceClient mission_done_client_;

    ros::ServiceServer goToPoseGoalSrv_;
    ros::ServiceServer openGripperSrv_;
    ros::ServiceServer closeGripperSrv_;

    dynamixel_workbench_msgs::DynamixelCommand dynamixel_srv_;
    geometry_msgs::Pose poseReference_;

    bool pose_;
    bool open_;
    bool close_;

    int open_position_;
    int close_position_;
};

UR5Move::UR5Move(ros::NodeHandle& nh)
    : nodeHandle_(nh), move_group_("manipulator"), pose_(false), open_(false), close_(false)
{

    dynamixel_client_ = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    mission_done_client_ = nh.serviceClient<std_srvs::Trigger>("/mission_done");


    goToPoseGoalSrv_ = nodeHandle_.advertiseService("/go_to_pose_goal", &UR5Move::goToPoseGoalCallback, this);


    openGripperSrv_ = nodeHandle_.advertiseService("/open_gripper", &UR5Move::openGripperCallback, this);
    closeGripperSrv_ = nodeHandle_.advertiseService("/close_gripper", &UR5Move::closeGripperCallback, this);

    dynamixel_srv_.request.id = 1;
    dynamixel_srv_.request.addr_name = "Goal_Position";

    open_position_ = 1500;
    close_position_ = 1000;

    move_group_.setStartStateToCurrentState();
    move_group_.clearPathConstraints();
}


bool UR5Move::goToPoseGoalCallback(ur5_cpp_controller::GoToPose::Request& req, ur5_cpp_controller::GoToPose::Response& res) {
    ROS_INFO("Received a pose goal. Moving to target...");

    poseReference_ = req.goal_pose; 
    pose_ = true;

    moveToPose();  

    res.success = true; 
    return true;
}

bool UR5Move::openGripperCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    ROS_INFO("Received request to open gripper.");
    open_ = true;

    openGripper();  

    return true;
}

bool UR5Move::closeGripperCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    ROS_INFO("Received request to close gripper.");
    close_ = true;

    closeGripper(); 

    return true;
}

void UR5Move::moveToPose() {
    ROS_INFO("Moving to Pose: [x: %f, y: %f, z: %f, orientation: x: %f, y: %f, z: %f, w: %f]",
             poseReference_.position.x, poseReference_.position.y, poseReference_.position.z,
             poseReference_.orientation.x, poseReference_.orientation.y, poseReference_.orientation.z, poseReference_.orientation.w);

    move_group_.setPoseTarget(poseReference_);
    move_group_.setGoalPositionTolerance(0.2);
    move_group_.setGoalOrientationTolerance(0.2);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode error_code = move_group_.plan(plan);

    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Planning successful, executing...");
        move_group_.move();
    } else {
        ROS_ERROR("Planning failed!");
    }

    pose_ = false;
}

void UR5Move::openGripper() {
    ROS_INFO("Opening gripper");

    dynamixel_srv_.request.value = open_position_;
    if (dynamixel_client_.call(dynamixel_srv_)) {
        ROS_INFO("Gripper opened successfully.");
    } else {
        ROS_ERROR("Failed to open gripper.");
    }

    open_ = false;
}

void UR5Move::closeGripper() {
    ROS_INFO("Closing gripper");

    dynamixel_srv_.request.value = close_position_;
    if (dynamixel_client_.call(dynamixel_srv_)) {
        ROS_INFO("Gripper closed successfully.");
    } else {
        ROS_ERROR("Failed to close gripper.");
    }

    close_ = false;
}

void UR5Move::control() {
    ros::Rate rate(10); 

    while (ros::ok()) {
        if (pose_) {
            moveToPose();
            std_srvs::Trigger trigger;
            mission_done_client_.call(trigger); 
        }
        else if (open_) {
            openGripper();
            std_srvs::Trigger trigger;
            mission_done_client_.call(trigger);  
        }
        else if (close_) {
            closeGripper();
            std_srvs::Trigger trigger;
            mission_done_client_.call(trigger);  
        }

        rate.sleep(); 
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur5_move");
    ros::NodeHandle nh;

    UR5Move ur5(nh);

    ros::AsyncSpinner spinner(0);  
    spinner.start();  

    ur5.control();

    return 0;
}
