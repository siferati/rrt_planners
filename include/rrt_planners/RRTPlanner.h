#pragma once

#include <nav_core/base_global_planner.h>
#include <random>
#include <mutex>
#include <atomic>
#include "rrt_planners/Node.h"

namespace rrt_planners
{

class RRTPlanner : public nav_core::BaseGlobalPlanner
{
public:
	// TODO dynamic reconfigure
	static constexpr int MAX_TREE_SIZE = 3000;
	static constexpr double GOAL_SAMPLE_CHANCE = 0.05;
	static constexpr double RRT_STEP_SIZE = 2.0;
	static constexpr double TURNING_RADIUS = 0.25;
	static constexpr double DUBINS_COL_STEP_SIZE = 0.5;
	static constexpr double DUBINS_PUB_STEP_SIZE = 0.05;
	static constexpr double GOAL_THRESHOLD = 2.0;
	static constexpr int TREE_PUBLISH_RATE = 5;

	virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

	virtual bool makePlan(
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& path) override;

	/**
	 * Checks if the given pose is in collision with any obstacles.
	 * 
	 * @param pose	The pose to evaluate.
	 * 
	 * @return 		True if the pose is in collision,
	 * 				False otherwise.
	 */ 
	bool is_pose_in_collision(const Pose& pose) const;

	/**
	 * Checks if the given path is in collision with any obstacles.
	 * 
	 * @param path	The path to evaluate.
	 * 
	 * @return		True if the path is in collision,
	 * 				False otherwise.
	 */ 
	bool is_path_in_collision(std::shared_ptr<DubinsPath> path) const;

	/**
	 * Samples a random pose from the obstacle-free space.
	 * 
	 * @return The sampled pose.
	 */ 
	Pose sample_random_pose();

	/**
	 * Computes the pose that is t distance away
	 * from the start of the given path.
	 *
	 * @param path	The path to evaluate.
	 * @param t		The distance to steer.
	 * 
	 * @return 		The computed pose.
	 */ 
	Pose steer(std::shared_ptr<DubinsPath> path, const double t) const;

	/**
	 * Computes the path between the two given poses.
	 * 
	 * @param begin	The start of the path.
	 * @param end 	The end of the path.
	 * 
	 * @return 		The computed path.
	 */ 
	std::shared_ptr<DubinsPath> compute_path(const Pose& begin, const Pose& end);

	/**
	 * Searches the tree for the nearest node to the given pose.
	 * 
	 * @param pose	Pose to evaluate.
	 * 
	 * @return		Nearest node in the tree.
	 */ 
	std::shared_ptr<Node> get_nearest_node(const Pose& pose) const;

	/**
	 * Creates a new node with the given pose and adds it to the tree.
	 * 
	 * @param pose		Pose of the new node.
	 * @param parent	Parent of the new node.
	 * @param edge		Edge from the parent node to the new node.
	 * 
	 * @return			The new node.
	 */ 
	virtual std::shared_ptr<Node> add_node(const Pose& pose, std::shared_ptr<Node> parent, std::shared_ptr<DubinsPath> edge);

	/**
	 * Reconnects the given node to the given parent if it's cheaper than current connection,
	 * and there are no obstacles in the way.
	 * 
	 * @param node		The node to reconnect.
	 * @param parent	The new parent of the node.
	 * 
	 * @return 			True if the node was reconnected,
	 * 					False otherwise.
	 */ 
	bool reconnect_node(std::shared_ptr<Node> node, std::shared_ptr<Node> parent);

	/**
	 * Retraces the path from the given node back to the root of the tree,
	 * and reverses it for convenience's sake.
	 * 
	 * @param node Node to start retracing from.
	 * @param path Output -- path found from the root of the tree to the given node.
	 * 
	 * @return True if a path exists, False otherwise.
	 */
	virtual bool retrace_path(std::shared_ptr<Node> node, std::vector<geometry_msgs::PoseStamped>& path);

	/**
	 * Publishes the given path.
	 * 
	 * @param path Path to publish.
	 */ 
	void publish_path(const std::vector<geometry_msgs::PoseStamped>& path) const;

	/**
	 * Callback for publishing the current tree on a fixed timer.
	 * 
	 * @param event	Information about the current callback event.
	 */ 
	void publish_tree_cb(const ros::TimerEvent& event);

	/**
	 * Clears the published markers.
	 */ 
	void clear_markers() const;

protected:
	/** Publisher for the tree. */
	ros::Publisher tree_pub;

	/** Timer for the publisher of the tree. */
	ros::Timer tree_pub_timer;

	/** Publisher for the plan. */
	ros::Publisher plan_pub;

	/** ROS wrapper for the 2d costmap. */
	costmap_2d::Costmap2DROS* costmap_ros;

	/** The 2d costmap. */
	costmap_2d::Costmap2D* costmap;

	/** List of nodes currently in the tree. */
	std::vector<std::shared_ptr<Node>> tree;

	/** Mutex for accessing the list of nodes. */
	std::mutex tree_mutex;

	/** True if currently planning, False otherwise. */
	std::atomic_bool is_planning;

	/** True if the last tree after planning finished was published, False otherwise. */
	std::atomic_bool is_last_tree_published;

	/** Random number generator. */
	std::mt19937 rng;

	/** Distribution for sampling random nodes. */
	std::array<std::uniform_real_distribution<double>, 3> pose_distribution;

	/** Distribution for sampling the goal node. */
	std::uniform_real_distribution<double> goal_sample_distribution;
};

}
