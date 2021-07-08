#include "rrt_planners/RRTStarPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(rrt_planners::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planners
{
void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	RRTPlanner::initialize(name, costmap_ros);
}

bool RRTStarPlanner::makePlan(
	const geometry_msgs::PoseStamped& start,
	const geometry_msgs::PoseStamped& goal,
	std::vector<geometry_msgs::PoseStamped>& path)
{		
	// thread safe
	{
		std::lock_guard<std::mutex> lock(this->tree_mutex);
		this->tree.clear();
		this->tree.reserve(MAX_TREE_SIZE);
		
		// delete markers
		this->clear_markers();
		ros::Duration(0.5).sleep();
	}

	// TODO transform to costmap frame

	// create goal node
	auto goal_node = std::make_shared<Node>(Pose(
		goal.pose.position.x,
		goal.pose.position.y,
		tf2::getYaw(goal.pose.orientation)));

	// create start node
	auto start_node = std::make_shared<Node>(Pose(
		start.pose.position.x,
		start.pose.position.y,
		tf2::getYaw(start.pose.orientation)));
	start_node->cost = 0;

	// add start node as the root tree
	this->tree.push_back(start_node);

	// signal planning start
	this->is_planning.store(true);

	while (tree.size() < MAX_TREE_SIZE)
	{
		// sample goal pose or random pose from free space
		Pose pose = this->goal_sample_distribution(this->rng) < GOAL_SAMPLE_CHANCE ? goal_node->pose : this->sample_random_pose();

		// compute edge to nearest node in the tree
		auto nearest = this->get_nearest_node(pose);
		auto edge = this->compute_path(nearest->pose, pose);

		// saturate
		if (dubins_path_length(edge.get()) > RRT_STEP_SIZE)
		{
			pose = this->steer(edge, RRT_STEP_SIZE);
			dubins_extract_subpath(edge.get(), RRT_STEP_SIZE, edge.get());
		}

		// TODO error on dubins return != 0

		// move on to next ite if edge is invalid
		if (this->is_path_in_collision(edge))
		{
			continue;
		}
		
		// add node to the tree
		auto node = this->add_node(pose, nearest, edge);

		// reconnect node along minimum-cost path
		auto near_nodes = this->get_nodes_in_radius(pose, SHRINKING_RADIUS);
		for (const auto& near_node : near_nodes)
		{
			this->reconnect_node(node, near_node);
		}

		// rewire the tree
		for (const auto& near_node : near_nodes)
		{
			this->reconnect_node(near_node, node);
		}

		// connect node to the goal if it's close enough
		if (node->pose.distance_to(goal_node->pose) < GOAL_THRESHOLD)
		{
			this->reconnect_node(goal_node, node);
		}
	}

	// signal planning end
	this->is_planning.store(false);

	if (this->retrace_path(goal_node, path))
	{
		this->publish_path(path);
		return true;
	}

	ROS_WARN("Could not compute a valid path!");

	return false;
}


std::vector<std::shared_ptr<Node>> RRTStarPlanner::get_nodes_in_radius(const Pose& pose, const double radius) const
{
	std::vector<std::shared_ptr<Node>> nodes;

	for (const auto& elem : this->tree)
	{
		double dist = pose.distance_to(elem->pose);
		if (dist < radius)
		{
			nodes.push_back(elem);
		}
	}

	return nodes;
}

}
