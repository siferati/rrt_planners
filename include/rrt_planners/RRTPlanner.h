#pragma once

#include <nav_core/base_global_planner.h>
#include <random>
#include "rrt_planners/Node.h"

namespace rrt_planners
{

class RRTPlanner : public nav_core::BaseGlobalPlanner
{
public:
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	bool makePlan(
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& plan);

private:
	/** ROS wrapper for the 2d costmap. */
	costmap_2d::Costmap2DROS* costmap_ros;

	/** The 2d costmap. */
	costmap_2d::Costmap2D* costmap;

	/** List of nodes currently in the tree. */
	std::vector<std::shared_ptr<Node>> tree;

	/** Random number generator. */
	std::mt19937 rng;

	/** Distribution for sampling random nodes. */
	std::array<std::uniform_real_distribution<double>, 3> pose_distribution;

	/** Find the nearest neighbour of the given node in the tree. */
	std::shared_ptr<Node> nearest_neighbour(std::shared_ptr<const Node> node) const;

	/** Computes the distance between the given nodes, multiplying heading distance by given weight. */
	double distance(std::shared_ptr<const Node> n1, std::shared_ptr<const Node> n2, double c = 1.0) const;

	/** Computes the linear interpolation between the given nodes for the given t. */ 
	std::shared_ptr<Node> lerp(std::shared_ptr<const Node> n1, std::shared_ptr<const Node> n2, const double t) const;

	/** Add the given node to the tree and set its parent. Returns true if successful, False otherwise. */
	bool add_node(std::shared_ptr<Node> node, std::shared_ptr<const Node> parent);
};

}
