/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Creates a cartesian path to be inserted into a planning roadmap
*/

#ifndef CURIE_DEMOS_CART_PATH_PLANNER_H
#define CURIE_DEMOS_CART_PATH_PLANNER_H

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// ROS
#include <ros/ros.h>

// Descartes
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <ur5_demo_descartes/ur5_robot_model.h>

// Eigen
#include <eigen_conversions/eigen_msg.h>

// OMPL
#include <ompl/tools/bolt/TaskGraph.h>

// this package
#include <curie_demos/imarker_robot_state.h>

namespace curie_demos
{
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";
const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
const double SERVICE_TIMEOUT = 5.0f;  // seconds
const double ORIENTATION_INCREMENT = 0.5f;
const double EPSILON = 0.0001f;
const double AXIS_LINE_LENGTH = 0.01;
const double AXIS_LINE_WIDTH = 0.001;
const double LINE_WIDTH = 0.005;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "home";

typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;

/*
 * Holds the data used at various points in the application.  This structure is populated
 * from data found in the ros parameter server at runtime.
 */
struct DemoConfiguration
{
  std::string group_name;               /* Name of the manipulation group containing the relevant links in the robot */
  std::string tip_link;                 /* Usually the last link in the kinematic chain of the robot */
  std::string base_link;                /* The name of the base link of the robot */
  std::string world_frame;              /* The name of the world link in the URDF file */
  std::vector<std::string> joint_names; /* A list with the names of the mobile joints in the robot */

  /* Trajectory Generation Members:
   *  Used to control the attributes (points, shape, size, etc) of the robot trajectory.
   *  */
  double time_delay;             /* Time step between consecutive points in the robot path */
  double foci_distance;          /* Controls the size of the curve */
  double radius;                 /* Controls the radius of the sphere on which the curve is projected */
  int num_points;                /* Number of points per curve */
  int num_lemniscates;           /* Number of curves*/
  std::vector<double> center;    /* Location of the center of all the lemniscate curves */
  std::vector<double> seed_pose; /* Joint values close to the desired start of the robot path */

  /*
   * Visualization Members
   * Used to control the attributes of the visualization artifacts
   */
  double min_point_distance; /* Minimum distance between consecutive trajectory points. */
};

class CurieDemos;

class CartPathPlanner
{
public:
  /**
   * \brief Constructor
   */
  CartPathPlanner(CurieDemos* parent);

  void processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                          const Eigen::Affine3d& feedback_pose);

  bool visualizeDescartesCartPath(const Eigen::Affine3d &start_pose);

  bool generateCartGraph(ompl::tools::bolt::TaskGraphPtr task_graph);

  void loadParameters();
  void initDescartes();
  bool generateCartTrajectory(DescartesTrajectory& traj, const Eigen::Affine3d &start_pose);
  void planPath(DescartesTrajectory& input_traj, DescartesTrajectory& output_path);
  moveit_msgs::RobotTrajectory runPath(const DescartesTrajectory& path);

  bool createLemniscateCurve(double foci_distance, double sphere_radius, int num_points, int num_lemniscates,
                                    const Eigen::Vector3d& sphere_center, EigenSTL::vector_Affine3d& poses);

  void fromDescartesToMoveitTraj(const DescartesTrajectory& in_traj, trajectory_msgs::JointTrajectory& out_traj);
  void fromDescartesToMoveitTraj(const DescartesTrajectory& in_traj, std::vector<moveit::core::RobotStatePtr> &out_traj);

  void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // Parent class
  CurieDemos* parent_;

  // State
  moveit::core::RobotStatePtr imarker_state_;

  // Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Trajetory
  //std::vector<moveit::core::RobotStatePtr> trajectory_;

  // Interactive markers
  IMarkerRobotStatePtr imarker_cartesian_;

  // Holds the data used by the various functions in the application.
  DemoConfiguration config_;

  // The planning group to work on
  moveit::core::JointModelGroup* jmg_;

  // Performs tasks specific to the Robot such IK, FK and collision detection*/
  descartes_core::RobotModelPtr ur5_robot_model_;

  // Plans a smooth robot path given a trajectory of points
  //descartes_planner::SparsePlanner planner_;
  descartes_planner::DensePlanner planner_;

  // User settings
  bool check_collisions_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<CartPathPlanner> CartPathPlannerPtr;
typedef boost::shared_ptr<const CartPathPlanner> CartPathPlannerConstPtr;

}  // namespace curie_demos
#endif  // CURIE_DEMOS_CART_PATH_PLANNER_H
