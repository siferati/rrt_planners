#include "rrt_planners/RRTXPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(rrt_planners::RRTXPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planners
{

void RRTXPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	RRTStarPlanner::initialize(name, costmap_ros);
}

bool RRTXPlanner::makePlan(
	const geometry_msgs::PoseStamped& start,
	const geometry_msgs::PoseStamped& goal,
	std::vector<geometry_msgs::PoseStamped>& path)
{
	// TODO reset when new goal is diff from previous goal

	// create goal node
	auto goal_node = std::make_shared<Node>(Pose(
		goal.pose.position.x,
		goal.pose.position.y,
		tf2::getYaw(goal.pose.orientation)));
	goal_node->lmc = 0;

	// create start node (backwards search)
	auto start_node = std::make_shared<Node>(Pose(
		start.pose.position.x,
		start.pose.position.y,
		tf2::getYaw(start.pose.orientation)));
	start_node->cost = 0;

	// add goal node as the root tree (backwards search)
	this->tree.push_back(goal_node);

	// TODO shrink radius

	// TODO update obstacles

	// TODO update robot

	// signal planning start
	this->is_planning.store(true);

	while (tree.size() < MAX_TREE_SIZE)
	{
		// sample start pose or random pose from free space
		Pose pose = this->goal_sample_distribution(this->rng) < GOAL_SAMPLE_CHANCE ? start_node->pose : this->sample_random_pose();

		// compute edge to nearest node in the tree
		auto nearest = this->get_nearest_node(pose);
		auto edge = this->compute_path(pose, nearest->pose);

		// saturate
		if (dubins_path_length(edge.get()) > RRT_STEP_SIZE)
		{
			pose = this->steer(edge, RRT_STEP_SIZE);
			dubins_extract_subpath(edge.get(), RRT_STEP_SIZE, edge.get());
		}

		// continue to next ite if pose is invalid
		if (this->is_pose_in_collision(pose)) continue;

		// add node to the tree
		auto node = this->add_node(pose);
		
		// continue to next ite if node is invalid
		if (node == nullptr) continue;


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


std::shared_ptr<Node> RRTXPlanner::add_node(const Pose& pose)
{
	auto node = std::make_shared<Node>(pose);
	auto neighbours = this->get_nodes_in_radius(pose, SHRINKING_RADIUS);
	
	// find the best parent
	for (const auto& neighbour : neighbours)
	{		
		auto edge = this->compute_path(pose, neighbour->pose);		
		double lmc = dubins_path_length(edge.get()) + neighbour->lmc;

		// best parent so far
		if (dubins_path_length(edge.get()) <= SHRINKING_RADIUS && node->lmc > lmc && !this->is_path_in_collision(edge))
		{
			node->parent = neighbour;
			node->lmc = lmc;
			node->edge = edge;
		}	
	}

	// valid parent found
	if (auto parent = node->parent)
	{
		// add node to the tree
		this->tree.push_back(node);
		parent->children.push_back(node);

		// update neighbours
		// TODO use edges calculated above to avoid duplicate computations
		for (const auto& neighbour : neighbours)
		{		
			// node -> neighbour
			auto edge = this->compute_path(pose, neighbour->pose);
			if (!this->is_path_in_collision(edge))
			{
				node->original_out_neighbours.insert({neighbour, edge});
				neighbour->running_in_neighbours.insert({node, edge});
			}

			// neighbour -> node
			edge = this->compute_path(neighbour->pose, pose);
			if (!this->is_path_in_collision(edge))
			{
				neighbour->running_out_neighbours.insert({node, edge});
				node->original_in_neighbours.insert({neighbour, edge});
			}
		}

		return node;
	}

	return nullptr;
}


void RRTXPlanner::cull_neighbours(std::shared_ptr<Node> node)
{
	auto it = std::begin(node->running_out_neighbours);
	while (it != std::end(node->running_out_neighbours))
	{		
		if (SHRINKING_RADIUS < dubins_path_length(it->second.get()) && node->parent != it->first)
		{
			it->first->running_in_neighbours.erase(node);
			it = node->running_out_neighbours.erase(it);
		}
		else
		{
			++it;
		}
	}
}


void RRTXPlanner::rewire_neighbours(std::shared_ptr<Node> node)
{
	if (node->cost - node->lmc <= EPSILON) return;

	this->cull_neighbours(node);

	// TODO HEEEEEEEEEERE rewire line 4-8
	auto rewire = [](const std::shared_ptr<Node>& neighbour)
	{

	};

	// rewire line 3
}

}
