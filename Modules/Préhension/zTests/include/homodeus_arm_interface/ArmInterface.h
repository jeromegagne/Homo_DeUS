#ifndef ARMINTERFACE_H
#define ARMINTERFACE_H

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



/* ArmInterface

Description:    This class acts as a wrapper to move TIAGo's arm
                easily with MoveIt.

Attributes:     _planningTime (type, float):
                    The time allowed to the planner to find a trajectory.
                    The default value is set to 5.0 seconds.

                _plannerId (type, std::string):
                    The planner's name that MoveIt uses to find a trajectory.
                    The default value is set to "SBLkConfigDefault".

                _jointsNames (type, std::vector<std::string>):
                    The name of every joints that MoveIt needs to move.

                _moveGroup (type, moveit::planning_interface::MoveGroupInterface):
                    Object to use the ROS interfaces provided by the move_group node.
*/
class ArmInterface
{
    private:
        float _planningTime;
        float max_vel_factor = 0.1;
        std::string _ref_frame;
        std::string _plannerId;
        std::vector<std::string> _jointsNames;
        moveit::planning_interface::MoveGroupInterface _moveGroup;
        // moveit::planning_interface::PlanningSceneInterface _planningScene;

        bool planTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan);
        bool planTrajectoryJ(moveit::planning_interface::MoveGroupInterface::Plan &plan);

    public:
        ArmInterface();
        ArmInterface(std::string ref_frame);

        void setPlanningTime(float value);
        void setPlannerId(std::string id);
        bool moveToCartesian(double x, double y, double z, double roll, double pitch, double yaw);
        bool moveToJoint(double torso, double j1, double j2, double j3, double j4, double j5, double j6, double j7);
};

#endif