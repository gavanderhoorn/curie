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

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// this package
#include <curie_demos/cart_path_planner.h>
#include <curie_demos/curie_demos.h>

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>

namespace curie_demos
{
CartPathPlanner::CartPathPlanner(CurieDemos* parent) : name_("cart_path_planner"), nh_("~"), parent_(parent)
{
  jmg_ = parent_->jmg_;

  // Load planning state
  imarker_state_.reset(new moveit::core::RobotState(*parent_->moveit_start_));

  // Create cartesian start pose interactive marker
  imarker_cartesian_.reset(new IMarkerRobotState(parent_->getPlanningSceneMonitor(), "cart", jmg_, parent_->ee_link_,
                                                 rvt::BLUE, parent_->package_path_));
  imarker_cartesian_->setIMarkerCallback(
      std::bind(&CartPathPlanner::processIMarkerPose, this, std::placeholders::_1, std::placeholders::_2));

  // Set visual tools
  visual_tools_ = imarker_cartesian_->getVisualTools();
  visual_tools_->setMarkerTopic(nh_.getNamespace() + "/cartesian_trajectory");
  visual_tools_->loadMarkerPub(true);
  visual_tools_->deleteAllMarkers();
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->loadTrajectoryPub(nh_.getNamespace() + "/display_trajectory");

  // Load Descartes ------------------------------------------------

  // loading parameters
  loadParameters();

  // initializing descartes
  initDescartes();

  ROS_INFO_STREAM_NAMED(name_, "CartPathPlanner Ready.");
}

void CartPathPlanner::processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                                         const Eigen::Affine3d& feedback_pose)
{
  imarker_state_ = imarker_cartesian_->getRobotState();
  Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);

  // visualizeMoveItCartPath(start_pose);
  visualizeDescartesCartPath(start_pose);
}

bool CartPathPlanner::visualizeDescartesCartPath(const Eigen::Affine3d& start_pose)
{
  ROS_DEBUG_STREAM_NAMED(name_, "visualizeDescartesCartPath()");

  // generating trajectory
  generateCartTrajectory(start_pose);

  return true;
}

bool CartPathPlanner::generateCartGraph()
{
  imarker_state_ = imarker_cartesian_->getRobotState();
  Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);

  // Generating trajectory
  bool debug = true;
  if (!generateCartTrajectory(start_pose, debug))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Failed to generate full cart trajectory at current position");
    return false;
  }

  // Benchmark runtime
  ros::Time start_time = ros::Time::now();

  const descartes_planner::PlanningGraph& pg = planner_.getPlanningGraph();

  // ------------------------------------------------------------------
  // Create the Descartes planning graph
  if (!planner_.insertGraph(cart_traj_))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Failed to create Descarrtes graph for trajectory");

    // Run again in debug mode
    bool debug = true;
    if (!generateCartTrajectory(start_pose, debug))
      return false;
  }

  double duration = (ros::Time::now() - start_time).toSec();
  ROS_INFO_STREAM_NAMED(name_, "Descartes graph generated in " << duration << " seconds with "
                                                               << boost::num_vertices(pg.getGraph()) << " vertices");
  std::cout << std::endl;

  return true;
}

bool CartPathPlanner::convertDescartesGraphToBolt(ompl::tools::bolt::TaskGraphPtr task_graph)
{
  std::size_t indent = 0;

  // Convert graph formats from Descartes to Bolt
  using namespace descartes_planner;
  using namespace descartes_core;
  using namespace descartes_trajectory;

  // Remove any previous Cartesian vertices/edges
  task_graph->generateTaskSpace(indent);

  ROS_INFO_STREAM_NAMED(name_, "Converting Descartes graph to Bolt graph");

  // For converting to MoveIt! format
  moveit::core::RobotStatePtr moveit_robot_state(new moveit::core::RobotState(*visual_tools_->getSharedRobotState()));
  std::vector<double> empty_seed;  // seed is required but not used by getNominalJointPose()
  moveit_ompl::ModelBasedStateSpacePtr space = parent_->space_;

  // Map from a Descartes vertex to a Bolt vertex
  std::map<JointGraph::vertex_descriptor, ompl::tools::bolt::TaskVertex> descarteToBoltVertex;

  ompl::tools::bolt::TaskVertex startingVertex = task_graph->getNumVertices() - 1;
  (void)startingVertex;  // prevent unused variable warning
  std::size_t new_vertex_count = 0;
  const ompl::tools::bolt::VertexType vertex_type = ompl::tools::bolt::CARTESIAN;
  const ompl::tools::bolt::VertexLevel level = 1;  // middle layer
  const PlanningGraph& pg = planner_.getPlanningGraph();

  // Iterate through vertices
  std::pair<VertexIterator, VertexIterator> vi = vertices(pg.getGraph());
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    JointGraph::vertex_descriptor jv = *vert_iter;

    // Get the joint values for this vertex and convert to a MoveIt! robot state
    TrajectoryPt::ID tajectory_id = pg.getGraph()[jv].id;
    const JointMap& joint_map = pg.getJointMap();
    std::vector<double> joints_pose;
    try
    {
      const JointTrajectoryPt& pt = joint_map.at(tajectory_id);
      pt.getNominalJointPose(empty_seed, *ur5_robot_model_, joints_pose);
    }
    catch (std::out_of_range)
    {
      ROS_WARN_STREAM_NAMED(name_, "Unable to find JointTrajectoryPtr in JointMap");
      return false;
    }

    // Copy vector into moveit format
    moveit_robot_state->setJointGroupPositions(jmg_, joints_pose);

    // Create new OMPL state
    ompl::base::State* ompl_state = space->allocState();

    // Convert the MoveIt! robot state into an OMPL state
    space->copyToOMPLState(ompl_state, *moveit_robot_state);

    // Add vertex to task graph
    descarteToBoltVertex[jv] = task_graph->addVertex(ompl_state, vertex_type, level, indent);

    new_vertex_count++;
  }
  ompl::tools::bolt::TaskVertex endingVertex = task_graph->getNumVertices() - 1;
  (void)endingVertex;  // prevent unused variable warning
  ROS_INFO_STREAM_NAMED(name_, "Added " << new_vertex_count << " new vertices");

  // Iterate through edges
  std::size_t new_edge_count = 0;
  std::pair<EdgeIterator, EdgeIterator> ei = edges(pg.getGraph());
  const ompl::tools::bolt::EdgeType edge_type = ompl::tools::bolt::eCARTESIAN;
  for (EdgeIterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
  {
    JointGraph::vertex_descriptor jv1 = source(*edge_iter, pg.getGraph());
    JointGraph::vertex_descriptor jv2 = target(*edge_iter, pg.getGraph());

    const ompl::tools::bolt::TaskVertex v1 = descarteToBoltVertex.at(jv1);
    const ompl::tools::bolt::TaskVertex v2 = descarteToBoltVertex.at(jv2);
    BOOST_ASSERT_MSG(v1 > startingVertex && v1 <= endingVertex, "Attempting to create edge with out of range vertex");
    BOOST_ASSERT_MSG(v2 > startingVertex && v2 <= endingVertex, "Attempting to create edge with out of range vertex");

    task_graph->addEdge(v1, v2, edge_type, indent);

    new_edge_count++;
  }
  ROS_DEBUG_STREAM_NAMED(name_, "Added " << new_edge_count << " new edges");

  // Connect Descartes graph to Bolt graph

  // Enumerate the start & end vertice descriptors
  std::vector<JointGraph::vertex_descriptor> start_points;
  pg.findStartVertices(start_points);
  std::vector<JointGraph::vertex_descriptor> goal_points;
  pg.findEndVertices(goal_points);

  // Track the shortest straight-line cost across any pair of start/goal points
  double shortest_path_across_cart = std::numeric_limits<double>::infinity();

  // force visualization
  // task_graph->visualizeTaskGraph_ = true;

  // Loop through all start points
  ROS_INFO_STREAM_NAMED(name_, "Connecting Descartes start points to TaskGraph");
  for (JointGraph::vertex_descriptor& jv : start_points)
  {
    // Get the corresponding BoltGraph vertex
    const ompl::tools::bolt::TaskVertex start_vertex = descarteToBoltVertex.at(jv);

    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level0 = 0;
    bool isStart = true;
    if (!task_graph->connectVertexToNeighborsAtLevel(start_vertex, level0, isStart, indent))
    {
      OMPL_WARN("Failed to connect Descartes start vertex %u", jv);
    }

    // Calculate the shortest straight-line distance across Descartes graph
    // Record min cost for cost-to-go heurstic distance function later
    // Check if this start vertex has has the shortest path across the Cartesian graph
    for (JointGraph::vertex_descriptor& goal_v : goal_points)
    {
      // Get the corresponding BoltGraph vertex
      const ompl::tools::bolt::TaskVertex goal_vertex = descarteToBoltVertex.at(goal_v);

      double distance_across_graph = task_graph->distanceFunction(start_vertex, goal_vertex);

      if (distance_across_graph < shortest_path_across_cart)
      {
        shortest_path_across_cart = distance_across_graph;
      }
    }

    if (!ros::ok())
      exit(-1);
  }

  // Loop through all goal points
  ROS_INFO_STREAM_NAMED(name_, "Connecting Descartes end points to TaskGraph");
  for (JointGraph::vertex_descriptor& jv : goal_points)
  {
    // Get the corresponding BoltGraph vertex
    const ompl::tools::bolt::TaskVertex goal_vertex = descarteToBoltVertex.at(jv);

    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level2 = 2;
    bool isStart = false;
    if (!task_graph->connectVertexToNeighborsAtLevel(goal_vertex, level2, isStart, indent))
    {
      OMPL_WARN("Failed to connect Descartes goal vertex %u", jv);
    }

    if (!ros::ok())
      exit(-1);
  }
  ROS_INFO_STREAM_NAMED(name_, "Finished connecting Descartes end points to TaskGraph");

  // Set the shortest path across cartesian graph in the TaskGraph
  task_graph->setShortestDistAcrossCart(shortest_path_across_cart);

  // Tell the planner to require task planning
  task_graph->setTaskPlanningEnabled();

  task_graph->printGraphStats();

  return true;
}

void CartPathPlanner::initDescartes()
{
  ROS_INFO_STREAM_NAMED(name_, "initDescartes()");

  // Instantiating a robot model
  const std::string prefix = "right_";
  ur5_robot_model_.reset(new ur5_demo_descartes::UR5RobotModel(prefix));

  // ur5_demo_descartes::UR5RobotModel ur5_robot_model_;

  // Initialize
  if (ur5_robot_model_->initialize(visual_tools_->getSharedRobotState()->getRobotModel(), config_.group_name,
                                   config_.world_frame, config_.tip_link))
  {
    ROS_INFO_STREAM("Descartes Robot Model initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Robot Model");
    exit(-1);
  }

  // Set collision checking.
  ur5_robot_model_->setCheckCollisions(check_collisions_);
  if (!check_collisions_)
    ROS_WARN_STREAM_NAMED(name_, "Collision checking disabled");

  bool succeeded = planner_.initialize(ur5_robot_model_);
  if (succeeded)
  {
    ROS_INFO_STREAM("Descartes Dense Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Dense Planner");
    exit(-1);
  }
}

void CartPathPlanner::loadParameters()
{
  ROS_INFO_STREAM_NAMED(name_, "loadParameters()");

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "group_name", config_.group_name);
  error += !rosparam_shortcuts::get(name_, rpnh, "tip_link", config_.tip_link);
  error += !rosparam_shortcuts::get(name_, rpnh, "base_link", config_.base_link);
  error += !rosparam_shortcuts::get(name_, rpnh, "world_frame", config_.world_frame);
  error += !rosparam_shortcuts::get(name_, rpnh, "check_collisions", check_collisions_);
  error += !rosparam_shortcuts::get(name_, rpnh, "orientation_increment", orientation_increment_);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/time_delay", config_.time_delay);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/foci_distance", config_.foci_distance);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/radius", config_.radius);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/num_points", config_.num_points);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/num_lemniscates", config_.num_lemniscates);
  rosparam_shortcuts::shutdownIfError(name_, error);
}

bool CartPathPlanner::createLemniscateCurve(double foci_distance, double sphere_radius, int num_points,
                                            int num_lemniscates, const Eigen::Vector3d& sphere_center,
                                            EigenSTL::vector_Affine3d& poses)
{
  ROS_DEBUG_STREAM_NAMED(name_, "createLemniscateCurve()");

  double a = foci_distance;
  double ro = sphere_radius;
  int npoints = num_points;
  int nlemns = num_lemniscates;
  Eigen::Vector3d offset(sphere_center[0], sphere_center[1], sphere_center[2]);
  Eigen::Vector3d unit_z, unit_y, unit_x;

  // checking parameters
  if (a <= 0 || ro <= 0 || npoints < 10 || nlemns < 1)
  {
    ROS_ERROR_STREAM("Invalid parameters for lemniscate curve were found");
    return false;
  }

  // generating polar angle values
  std::vector<double> theta(npoints);

  // interval 1 <-pi/4 , pi/4 >
  double d_theta = 2 * M_PI_2 / (npoints - 1);
  for (unsigned int i = 0; i < static_cast<std::size_t>(npoints) / 2; i++)
  {
    theta[i] = -M_PI_4 + i * d_theta;
  }
  theta[0] = theta[0] + EPSILON;
  theta[npoints / 2 - 1] = theta[npoints / 2 - 1] - EPSILON;

  // interval 2 < 3*pi/4 , 5 * pi/4 >
  for (unsigned int i = 0; i < static_cast<std::size_t>(npoints) / 2; i++)
  {
    theta[npoints / 2 + i] = 3 * M_PI_4 + i * d_theta;
  }
  theta[npoints / 2] = theta[npoints / 2] + EPSILON;
  theta[npoints - 1] = theta[npoints - 1] - EPSILON;

  // generating omega angle (lemniscate angle offset)
  std::vector<double> omega(nlemns);
  double d_omega = M_PI / (nlemns);
  for (unsigned int i = 0; i < static_cast<std::size_t>(nlemns); i++)
  {
    omega[i] = i * d_omega;
  }

  Eigen::Affine3d pose;
  double x, y, z, r, phi;

  poses.clear();
  poses.reserve(nlemns * npoints);
  for (unsigned int j = 0; j < static_cast<std::size_t>(nlemns); j++)
  {
    for (unsigned int i = 0; i < static_cast<std::size_t>(npoints); i++)
    {
      r = std::sqrt(std::pow(a, 2) * std::cos(2 * theta[i]));
      phi = r < ro ? std::asin(r / ro) : (M_PI - std::asin((2 * ro - r) / ro));

      x = ro * std::cos(theta[i] + omega[j]) * std::sin(phi);
      y = ro * std::sin(theta[i] + omega[j]) * std::sin(phi);
      z = ro * std::cos(phi);

      // determining orientation
      unit_z << -x, -y, -z;
      unit_z.normalize();

      unit_x = (Eigen::Vector3d(0, 1, 0).cross(unit_z)).normalized();
      unit_y = (unit_z.cross(unit_x)).normalized();

      Eigen::Matrix3d rot;
      rot << unit_x(0), unit_y(0), unit_z(0), unit_x(1), unit_y(1), unit_z(1), unit_x(2), unit_y(2), unit_z(2);

      pose = Eigen::Translation3d(offset(0) + x, offset(1) + y, offset(2) + z) * rot;

      // DTC: Rotate -90 so that the x axis points down
      pose = pose * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY());

      poses.push_back(pose);
    }
  }

  return true;
}

bool CartPathPlanner::createDrawing(const Eigen::Vector3d& starting_point, EigenSTL::vector_Affine3d& poses)
{
  ROS_DEBUG_STREAM_NAMED(name_, "createDrawing()");

  poses.clear();

  Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
  start_pose.translation() = starting_point;

  // Rotate 90 so that the x axis points down
  start_pose = start_pose * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  // Rotate -
  // start_pose = start_pose * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY());

  poses.push_back(start_pose);

  const double distance = config_.foci_distance;
  const std::size_t increments = config_.num_points;
  const double step = distance / static_cast<double>(increments);
  Eigen::Affine3d end_pose = start_pose;
  for (std::size_t i = 0; i < increments; ++i)
  {
    end_pose.translation().x() += step;
    poses.push_back(end_pose);
  }

  return true;
}

bool CartPathPlanner::generateCartTrajectory(const Eigen::Affine3d& start_pose, bool debug)
{
  ROS_DEBUG_STREAM_NAMED(name_, "generateCartTrajectory()");
  if (debug)
    ROS_WARN_STREAM_NAMED(name_, "Running in debug mode");

  EigenSTL::vector_Affine3d exact_poses;

  Eigen::Vector3d sphere_center = start_pose.translation();
  sphere_center.z() -= 0.05;  // move center down a bit from end effector TODO(davetcoleman): remove hack

  // if (createLemniscateCurve(config_.foci_distance, config_.radius, config_.num_points, config_.num_lemniscates,
  //                           sphere_center, exact_poses))
  if (!createDrawing(sphere_center, exact_poses))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Trajectory generation failed");
    exit(-1);
  }

  ROS_DEBUG_STREAM_NAMED(name_, "Trajectory with " << exact_poses.size() << " points was generated");

  // Publish trajectory poses for visualization
  visual_tools_->deleteAllMarkers();
  visual_tools_->publishPath(exact_poses, rvt::ORANGE, rvt::XXSMALL);
  visual_tools_->publishAxisPath(exact_poses, rvt::XXXSMALL);
  visual_tools_->triggerBatchPublish();

  // return useDescartesToGetPoses(exact_poses, debug);

  // Specify tolerance
  OrientationTol orientation_tol(M_PI, M_PI / 10, M_PI / 10);

  if (debug)
    debugShowAllIKSolutions(exact_poses, orientation_tol);

  return true;
}

bool CartPathPlanner::useDescartesToGetPoses(EigenSTL::vector_Affine3d exact_poses, bool debug)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  // Create descartes trajectory points
  cart_traj_.clear();
  cart_traj_.reserve(exact_poses.size());
  for (unsigned int i = 0; i < exact_poses.size(); i++)
  {
    // Use AxialSymetricPt objects to allow a trajectory cartesian point with rotational freedom about the tool's z axis
    using namespace descartes_core;
    cart_traj_.push_back(TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(
        exact_poses[i], orientation_increment_, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS,
        TimingConstraint(0.5))));

    // X_AXIS - rotates around orange line
    // Y_AXIS - rotates long ways around orange line
    // Z_AXIS - rotates around orange line
  }

  if (debug)
  {
    // Enumerate the potential points
    // for (descartes_core::TrajectoryPtPtr cart_point : cart_traj_)
    for (std::size_t i = 0; i < cart_traj_.size(); ++i)
    {
      descartes_core::TrajectoryPtPtr cart_point = cart_traj_[i];
      Eigen::Affine3d& pose = exact_poses[i];

      // Compute all poses for each pose
      EigenSTL::vector_Affine3d poses;
      if (!dynamic_cast<CartTrajectoryPt*>(cart_point.get())->computeCartesianPoses(poses))
      {
        ROS_ERROR("Failed for find ANY cartesian poses");
        return false;
      }

      for (const auto& pose : poses)
      {
        visual_tools_->publishAxis(pose);
      }

      std::vector<std::vector<double>> joint_poses;
      cart_point->getJointPoses(*ur5_robot_model_, joint_poses);

      if (joint_poses.empty())
      {
        ROS_ERROR_STREAM_NAMED(name_, "No joint solutions found for pose " << i);

        visual_tools_->publishAxis(pose, rvt::XXSMALL);
        visual_tools_->triggerBatchPublish();

        // Show previous joint poses
        if (i > 0)
        {
          ROS_INFO_STREAM_NAMED(name_, "Showing last valid robot state in red");
          descartes_core::TrajectoryPtPtr cart_point = cart_traj_[i - 1];

          std::vector<std::vector<double>> joint_poses;
          cart_point->getJointPoses(*ur5_robot_model_, joint_poses);
          visual_tools_->publishRobotState(joint_poses.front(), jmg_, rvt::RED);
        }

        return false;
      }

      // Show first configuration
      if (false)
      {
        visual_tools_->publishRobotState(joint_poses.front(), jmg_);
        ros::Duration(0.1).sleep();
      }

      // Show all possible configurations
      if (true)
      {
        for (std::vector<double> pose : joint_poses)
        {
          visual_tools_->publishRobotState(pose, jmg_);
          ros::Duration(0.1).sleep();

          if (!ros::ok())
            exit(0);
        }
      }

      visual_tools_->publishAxis(pose, rvt::XXXXSMALL);
      visual_tools_->triggerBatchPublish();
    }
  }

  // ROS_INFO_STREAM_NAMED(name_, "getAllIK found all solutions for trajectory");
  return true;
}

bool CartPathPlanner::debugShowAllIKSolutions(const EigenSTL::vector_Affine3d& exact_poses,
                                              const OrientationTol& orientation_tol)
{
  // Enumerate the potential poses within tolerance
  for (std::size_t i = 0; i < exact_poses.size(); ++i)
  {
    const Eigen::Affine3d& pose = exact_poses[i];

    EigenSTL::vector_Affine3d candidate_poses;
    computeAllPoses(pose, orientation_tol, candidate_poses);

    std::vector<std::vector<double>> joint_poses;
    for (const Eigen::Affine3d& candidate_pose : candidate_poses)
    {
      std::vector<std::vector<double>> local_joint_poses;
      if (ur5_robot_model_->getAllIK(candidate_pose, local_joint_poses))
      {
        joint_poses.insert(joint_poses.end(), local_joint_poses.begin(), local_joint_poses.end());
      }
    }

    // Handle error: no IK solutions found
    if (joint_poses.empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "No joint solutions found for pose " << i);

      visual_tools_->publishAxis(pose, rvt::XXSMALL);
      visual_tools_->triggerBatchPublish();

      return false;
    }

    // Show all possible configurations
    if (true)
    {
      for (std::vector<double> joint_pose : joint_poses)
      {
        visual_tools_->publishRobotState(joint_pose, jmg_);
        //ros::Duration(0.1).sleep();

        if (!ros::ok())
          exit(0);
      }
    }

  }

  return true;
}

bool CartPathPlanner::computeAllPoses(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol,
                                      EigenSTL::vector_Affine3d& candidate_poses)
{
  BOOST_ASSERT_MSG(orientation_increment_ != 0, "Divide by zero using orientation increment");

  const std::size_t num_axis = 3;

  // Reserve vector size
  std::vector<size_t> num_steps_per_axis(num_axis, 0 /* value */);
  for (std::size_t i = 0; i < num_axis; ++i)
  {
    double range = 2 * orientation_tol.axis_dist_from_center_[i];
    num_steps_per_axis[i] = std::max(1.0, ceil(range / orientation_increment_));
  }
  double total_num_steps = num_steps_per_axis[0] * num_steps_per_axis[1] * num_steps_per_axis[2];
  std::cout << "Reserving space for " << total_num_steps << " poses" << std::endl;
  candidate_poses.reserve(total_num_steps);

  // Start recursion
  return rotateOnAxis(pose, orientation_tol, X_AXIS, candidate_poses);
}

bool CartPathPlanner::rotateOnAxis(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol,
                                         const Axis axis, EigenSTL::vector_Affine3d& candidate_poses)
{
  const bool verbose = false;
  double range = 2 * orientation_tol.axis_dist_from_center_[axis];
  std::size_t num_steps = std::max(1.0, ceil(range / orientation_increment_));
  Eigen::Affine3d new_pose;

  // Rotate all around one axis
  for (int i = num_steps * -0.5; i < num_steps * 0.5; ++i)
  {
    double rotation_amount = i * orientation_increment_;
    if (verbose)
      std::cout << std::string(axis * 2, ' ') << "axis: " << axis << " i: " << i
                << " rotation_amount: " << rotation_amount << " num_steps: " << num_steps << std::endl;

    // clang-format off
    switch (axis)
    {
      case X_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitX()); break;
      case Y_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitY()); break;
      case Z_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitZ()); break;
      default: ROS_ERROR_STREAM_NAMED(name_, "Unknown axis");
    }
    // clang-format on

    // Recursively rotate
    if (axis < Z_AXIS)
    {
      rotateOnAxis(new_pose, orientation_tol, static_cast<Axis>(axis + 1), candidate_poses);
    }
    else
    {
      candidate_poses.push_back(new_pose);
      // visual_tools_->publishZArrow(new_pose, rvt::BLUE, rvt::XXXSMALL);
      visual_tools_->publishAxis(new_pose, rvt::XXXSMALL);
    }
  }

  return true;
}


}  // namespace curie_demos
