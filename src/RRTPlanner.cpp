#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include "rrt_planners/RRTPlanner.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#define MAX_TREE_SIZE 100
#define EPSILON 0.01
#define STEP_SIZE 0.25
#define TURNING_RADIUS 0.1
#define DUBINS_STEP_SIZE 0.1
#define DUBINS_PUB_STEP_SIZE 0.05
#define GOAL_THRESHOLD 0.2
#define PUBLISH_RATE 30

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

	ros::NodeHandle nh;
	this->plan_pub = nh.advertise<nav_msgs::Path>("plan", 10);
	this->tree_pub = nh.advertise<visualization_msgs::Marker>("rapidly_exploring_random_tree", 10);
	//this->tree_pub_timer = nh.createTimer(ros::Duration(1 / PUBLISH_RATE), &RRTPlanner::publish_tree, this);
}


bool RRTPlanner::makePlan(
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& path)
{		
	this->tree.clear();
	this->tree.reserve(MAX_TREE_SIZE);

	// create goal node
	auto goal_node = std::make_shared<Node>(
		goal.pose.position.x,
		goal.pose.position.y,
		tf2::getYaw(start.pose.orientation));

	// add start to tree
	this->tree.push_back(std::make_shared<Node>(
		Pose(
			start.pose.position.x,
			start.pose.position.y,
			tf2::getYaw(start.pose.orientation)),
		0));

	while (tree.size() < MAX_TREE_SIZE)
	{
		// sample random pose from free space
		Pose pose = this->sample_random_pose();
		double pose_array[] = {pose.x, pose.y, pose.yaw};

		// get nearest node in the tree
		auto nearest = this->get_nearest_node(pose);
		double nearest_array[] = {nearest->pose.x, nearest->pose.y, nearest->pose.yaw};

		// tree edge is the shortest dubins path between the two poses
		DubinsPath edge;		
		dubins_shortest_path(&edge, nearest_array, pose_array, TURNING_RADIUS);

		// saturate
		if (dubins_path_length(&edge) > STEP_SIZE)
		{
			pose = this->steer(edge, STEP_SIZE);
			dubins_extract_subpath(&edge, STEP_SIZE, &edge);
		}

		// TODO collision check edge
		// TODO error on dubins return != 0
		
		// add node to the tree
		if (this->add_node(node, nearest))
		{
			if (this->distance(node, goal_node, 0) < GOAL_THRESHOLD)
			{
				this->add_node(goal_node, node);
			}
		}
	}

	if (this->retrace_path(goal_node, path))
	{
		this->publish_path(path);
		return true;
	}

	return false;
}


bool RRTPlanner::is_pose_in_collision(const Pose& pose) const
{
	unsigned int map_x, map_y;
	if (!costmap->worldToMap(pose.x, pose.y, map_x, map_y))
	{
		ROS_ERROR("Invalid conversion from world coordinates to map coordinates.");
		return true;
	}
	
	// TODO check footprint when possibly circumscribed
	unsigned char cost = costmap->getCost(map_x, map_y);
	if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
		cost == costmap_2d::LETHAL_OBSTACLE)
	{
		return true;
	}

	return false;
}


Pose RRTPlanner::sample_random_pose() const
{
	Pose pose;
	do
	{
		pose.x = this->pose_distribution[0](this->rng);
		pose.y = this->pose_distribution[0](this->rng);
		pose.yaw = this->pose_distribution[0](this->rng);
	}
	while (this->is_pose_in_collision(pose));
	
	return pose;
}


Pose RRTPlanner::steer(DubinsPath& path, const double t) const
{	
	double pose_array[3];
	dubins_path_sample(&path, t, pose_array);

	return Pose(pose_array[0], pose_array[1], pose_array[2]);
}


std::shared_ptr<Node> RRTPlanner::get_nearest_node(const Pose& pose) const
{
	std::shared_ptr<Node> nearest = nullptr;
	double min_dist = std::numeric_limits<double>::max();

	for (const auto& elem : tree)
	{
		double dist = pose.distance_to(elem->pose);
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest = elem;
		}
	}

	return nearest;
}


bool RRTPlanner::add_node(std::shared_ptr<Node> node, std::shared_ptr<Node> parent)
{	
	double q0[] = {parent->x, parent->y, parent->w};
	double q1[] = {node->x, node->y, node->w};

	// check if given pose is valid
	// 0: OK | 1: OBSTACLE | -1: ERROR
	auto check_pose = [](double q[3], double t, void* data) -> int
	{
		auto costmap = static_cast<costmap_2d::Costmap2D*>(data);

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
			return 1;
		}

		return 0;
	};

	// check if node is valid if it isn't already in the tree
	if (node->parent == nullptr && check_pose(q1, 1, this->costmap) != 0)
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
	if (dubins_path_sample_many(edge.get(), DUBINS_STEP_SIZE, check_pose, this->costmap) != 0)
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

void RRTPlanner::publish_path(const std::vector<geometry_msgs::PoseStamped>& path)
{
	nav_msgs::Path msg;
	msg.header.frame_id = this->costmap_ros->getGlobalFrameID();
	msg.header.stamp = ros::Time::now();
	msg.poses = path;

	this->plan_pub.publish(msg);
}

bool RRTPlanner::retrace_path(std::shared_ptr<Node> node, std::vector<geometry_msgs::PoseStamped>& path)
{	
	if (node == nullptr || node->parent == nullptr) return false;
	
	while (node->parent != nullptr)
	{
		double q[3];
		double t = dubins_path_length(node->edge.get());
		while(t > 0)
		{
			dubins_path_sample(node->edge.get(), t, q);

			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = this->costmap_ros->getGlobalFrameID();
			pose.pose.position.x = q[0];
			pose.pose.position.y = q[1];
			tf2::Quaternion quat;
			quat.setRPY(0, 0, q[3]);
			pose.pose.orientation = tf2::toMsg(quat);
			path.push_back(pose);

			t -= DUBINS_PUB_STEP_SIZE;
		}

		// t never reaches 0, so we add the parent here
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = this->costmap_ros->getGlobalFrameID();
		pose.pose.position.x = node->parent->x;
		pose.pose.position.y = node->parent->y;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, node->parent->w);
		pose.pose.orientation = tf2::toMsg(quat);
		path.push_back(pose);

		node = node->parent;
	}

	std::reverse(std::begin(path), std::end(path));

	return true;
}

/*void RRTPlanner::publish_path(std::shared_ptr<Node> node)
{
	visualization_msgs::Marker msg;
	msg.header.frame_id = this->costmap_ros->getGlobalFrameID();
	msg.header.stamp = ros::Time::now();
	msg.ns = "rapidly_exploring_random_tree";
	msg.id = 0;
	msg.action = visualization_msgs::Marker::ADD;
	msg.type = visualization_msgs::Marker::LINE_STRIP;
	msg.scale.x = 0.01;
	msg.color.g = 1;
	msg.color.a = 0.5;
	msg.pose.orientation.w = 1;

	while (node->parent != nullptr)
	{
		double q[3];
		double t = dubins_path_length(node->edge.get());
		while(t > 0)
		{
			dubins_path_sample(node->edge.get(), t, q);

			geometry_msgs::Point p;
			p.x = q[0];
			p.y = q[1];
			msg.points.push_back(p);

			t -= DUBINS_PUB_STEP_SIZE;
		}

		// t never reaches 0, so we add the parent here
		geometry_msgs::Point p;
		p.x = node->parent->x;
		p.y = node->parent->y;
		msg.points.push_back(p);

		node = node->parent;
	}

	this->tree_pub.publish(msg);
}*/

}
