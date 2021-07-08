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
	// TODO dynamic rrtx

	// thread safe
	{
		std::lock_guard<std::mutex> lock(this->tree_mutex);
		this->tree.clear();
		this->tree.reserve(MAX_TREE_SIZE);
		
		// delete markers
		this->clear_markers();
		ros::Duration(0.5).sleep();
	}

	// node comparator for the queue
	auto comp = [](std::shared_ptr<Node> n1, std::shared_ptr<Node> n2)
	{
		double f1 = std::min(n1->cost, n1->lmc);
		double f2 = std::min(n2->cost, n2->lmc);

		return f1 < f2 || (f1 == f2 && n1->cost < n2->cost);
	};

	// create the priority queue
	this->inconsistent_nodes = std::set<
		std::shared_ptr<Node>,
		std::function<bool(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2)>
	>(comp);

	// create goal node
	auto goal_node = std::make_shared<Node>(Pose(
		goal.pose.position.x,
		goal.pose.position.y,
		tf2::getYaw(goal.pose.orientation)));
	goal_node->cost = 0;
	goal_node->lmc = 0;

	// create start node (backwards search)
	auto start_node = std::make_shared<Node>(Pose(
		start.pose.position.x,
		start.pose.position.y,
		tf2::getYaw(start.pose.orientation)));

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
		double length = dubins_path_length(edge.get());
		if (length > RRT_STEP_SIZE)
		{
			// TODO fix this -- in some cases the new edge does not match the old edge
			// we should be extracting the subpath [t: end] from the already computed edge
			// but the helper function only handles extracing the subpath [start: t]
			pose = this->steer(edge, length - RRT_STEP_SIZE);
			edge = this->compute_path(pose, nearest->pose);
		}

		// continue to next ite if pose is invalid
		if (this->is_pose_in_collision(pose))
		{
			continue;
		}

		// add node to the tree
		auto node = this->add_node(pose);
		
		// new node was successfully added
		if (node != nullptr)
		{
			this->rewire_neighbours(node);
			this->reduce_inconsistency();

			// TODO connect to start if close enough
		}
	}

	// signal planning end
	this->is_planning.store(false);

	// TODO fix retrace
	if (this->retrace_path(start_node, path))
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

	auto rewire = [&](const std::pair<std::shared_ptr<Node>, std::shared_ptr<DubinsPath>> elem)
	{
		const auto& neighbour = elem.first;
		const auto& edge = elem.second;

		// do not rewire the parent
		if (neighbour == node->parent) return;

		double lmc = dubins_path_length(edge.get()) + node->lmc;

		// rewire neighbour if better
		if (neighbour->lmc > lmc)
		{
			// make parent
			neighbour->lmc = lmc;
			neighbour->parent = node;
			neighbour->edge = edge;

			// check for e-consistency
			if (neighbour->cost - lmc > EPSILON)
			{
				// verify queue
				this->inconsistent_nodes.erase(neighbour);
				this->inconsistent_nodes.insert(neighbour);
			}
		}
	};

	this->cull_neighbours(node);

	std::for_each(
		std::begin(node->original_in_neighbours),
		std::end(node->original_in_neighbours),
		rewire
	);

	std::for_each(
		std::begin(node->running_in_neighbours),
		std::end(node->running_in_neighbours),
		rewire
	);
}


void RRTXPlanner::reduce_inconsistency()
{
	// TODO stop conditions related to vbot
	while (!this->inconsistent_nodes.empty())
	{
		// pop
		auto it = this->inconsistent_nodes.begin();
		this->inconsistent_nodes.erase(it);
		auto node = *it;

		// check for e-consistency
		if (node->cost - node->lmc < EPSILON)
		{
			this->update_lmc(node);
			this->rewire_neighbours(node);
		}

		// node is locally 0-consistent
		node->cost = node->lmc;
	}
}

void RRTXPlanner::update_lmc(std::shared_ptr<Node> node)
{
	this->cull_neighbours(node);

	auto rewire = [&](const std::pair<std::shared_ptr<Node>, std::shared_ptr<DubinsPath>> elem)
	{
		const auto& neighbour = elem.first;
		const auto& edge = elem.second;

		// do not rewire the parent
		if (neighbour->parent == node) return;

		double lmc = dubins_path_length(edge.get()) + neighbour->lmc;

		// rewire node if better
		if (node->lmc > lmc)
		{
			// make parent
			node->lmc = lmc;
			node->parent = neighbour;
			node->edge = edge;
		}
	};

	std::for_each(
		std::begin(node->original_out_neighbours),
		std::end(node->original_out_neighbours),
		rewire
	);

	std::for_each(
		std::begin(node->running_out_neighbours),
		std::end(node->running_out_neighbours),
		rewire
	);
}

}
