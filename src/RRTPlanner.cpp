#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include "rrt_planners/RRTPlanner.h"

#define MAX_TREE_SIZE 100
#define EPSILON 0.01
#define STEP_SIZE 0.5
#define TURNING_RADIUS 0.5
#define DUBINS_STEP_SIZE 0.1
#define GOAL_THRESHOLD 0.2

PLUGINLIB_EXPORT_CLASS(rrt_planners::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planners
{

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	this->costmap_ros = costmap_ros;
	this->costmap = costmap_ros->getCostmap();

	double resolution = this->costmap->getResolution();
	double size_x = this->costmap->getSizeInCellsX() * resolution;
	double size_y = this->costmap->getSizeInCellsY() * resolution;
	
	std::random_device seed;
	this->rng = std::mt19937(seed());
	this->pose_distribution = {
		std::uniform_real_distribution<double>(0, nextafter(size_x, std::numeric_limits<double>::max())),
		std::uniform_real_distribution<double>(0, nextafter(size_y, std::numeric_limits<double>::max())),
		std::uniform_real_distribution<double>(-M_PI, nextafter(M_PI, std::numeric_limits<double>::max()))
	};
}


bool RRTPlanner::makePlan(
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& plan)
{		
	this->tree.clear();
	this->tree.reserve(MAX_TREE_SIZE);

	// create goal node
	auto goal_node = std::make_shared<Node>(
		goal.pose.position.x,
		goal.pose.position.y,
		tf::getYaw(start.pose.orientation));

	// add start to tree
	this->tree.push_back(std::make_shared<Node>(
		start.pose.position.x,
		start.pose.position.y,
		tf::getYaw(start.pose.orientation)));

	while (tree.size() < MAX_TREE_SIZE)
	{	
		// sample random node
		double x = pose_distribution[0](this->rng);
		double y = pose_distribution[0](this->rng);
		double w = pose_distribution[0](this->rng);
		auto node = std::make_shared<Node>(x, y, w);

		auto nearest = this->nearest_neighbour(node);
		double dist = this->distance(node, nearest);

		// saturate
		if (dist > STEP_SIZE)
		{
			node = this->lerp(nearest, node, STEP_SIZE / dist);
			dist = STEP_SIZE;
		}
		
		// add node to the tree
		if (this->add_node(node, nearest))
		{
			if (this->distance(node, goal_node, 0) < GOAL_THRESHOLD)
			{
				this->add_node(goal_node, node);
			}
		}
	}

	// no path found
	if (goal_node->parent == nullptr)
	{
		return false;
	}

	auto node = goal_node->parent;
	while (node != nullptr)
	{
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = node->x;
		pose.pose.position.y = node->y;
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(node->w);
		plan.push_back(pose);

		node = node->parent;
	}

	return true;
}

std::shared_ptr<Node> RRTPlanner::nearest_neighbour(std::shared_ptr<const Node> node) const
{
	std::shared_ptr<Node> nearest = nullptr;
	double min_dist = std::numeric_limits<double>::max();

	for (const auto& elem : tree)
	{
		double dist = this->distance(node, elem);
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest = elem;
		}
	}

	return nearest;
}

double RRTPlanner::distance(std::shared_ptr<const Node> n1, std::shared_ptr<const Node> n2, double c) const
{
	return sqrt(c * pow(n1->w - n2->w, 2) + pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));
}

std::shared_ptr<Node> RRTPlanner::lerp(std::shared_ptr<const Node> n1, std::shared_ptr<const Node> n2, const double t) const
{
	return std::make_shared<Node>(
		n1->x + t*(n2->x - n1->x),
		n1->y + t*(n2->y - n1->y),
		n1->w + t*(n2->w - n1->w));
}

bool RRTPlanner::add_node(std::shared_ptr<Node> node, std::shared_ptr<const Node> parent)
{	
	double q0[] = {parent->x, parent->y, parent->w};
	double q1[] = {node->x, node->y, node->w};

	// check if given pose is valid
	auto is_valid = [](double q[3], double t, void* data) -> int
	{
		auto costmap = static_cast<costmap_2d::Costmap2D*>(data);
		std::cout << costmap->getOriginX() << std::endl;

		std::cout << "olaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
		std::cout << q[0] << ", " << q[1] << ", " << q[1] << std::endl;
		std::cout << "olaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;

		unsigned int map_x, map_y;
		if (!costmap->worldToMap(q[0], q[1], map_x, map_y))
		{
			ROS_ERROR("Invalid conversion from world coordinates to map coordinates.");
			return -1;
		}

		// collision check
		unsigned char cost = costmap->getCost(map_x, map_y);
		if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
			cost == costmap_2d::LETHAL_OBSTACLE)
		{
			return -1;
		}

		return 0;
	};

	// check if node is valid if it isn't already in the tree
	if (node->parent == nullptr && !is_valid(q1, 1, this->costmap))
	{
		return false;
	}

	// tree edge is shortest dubins path between node and parent
	auto edge = std::make_unique<DubinsPath>();
	dubins_shortest_path(edge.get(), q0, q1, TURNING_RADIUS);

	double cost = dubins_path_length(edge.get());

	// no need to evaluate this path if it's more expensive
	if (parent->cost + cost >= node->cost)
	{
		return false;
	}

	// check edge for collisions
	if (dubins_path_sample_many(edge.get(), DUBINS_STEP_SIZE, is_valid, this->costmap) != 0)
	{
		return false;
	}

	// add new node to the tree
	node->parent = parent;
	node->cost = parent->cost + cost;
	node->edge = std::move(edge);
	this->tree.push_back(node);
	
	return true;
}

}
