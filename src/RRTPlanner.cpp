#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include "rrt_planners/RRTPlanner.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

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
	this->goal_sample_distribution = std::uniform_real_distribution<double>(0, 1);
	this->pose_distribution = {
		std::uniform_real_distribution<double>(0, nextafter(size_x, std::numeric_limits<double>::max())),
		std::uniform_real_distribution<double>(0, nextafter(size_y, std::numeric_limits<double>::max())),
		std::uniform_real_distribution<double>(-M_PI, nextafter(M_PI, std::numeric_limits<double>::max()))
	};

	ros::NodeHandle nh;
	this->plan_pub = nh.advertise<nav_msgs::Path>("plan", 10);
	this->tree_pub = nh.advertise<visualization_msgs::Marker>("rapidly_exploring_random_tree", 10);
	this->tree_pub_timer = nh.createTimer(ros::Duration(1 / TREE_PUBLISH_RATE), &RRTPlanner::publish_tree_cb, this);
}


bool RRTPlanner::makePlan(
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& path)
{		
	// thread safe
	{
		std::lock_guard<std::mutex> lock(this->tree_mutex);
		this->tree.clear();
		this->tree.reserve(MAX_TREE_SIZE);
		this->clear_markers();
	}

	// TODO transform to costmap frame

	// create goal node
	auto goal_node = std::make_shared<Node>(Pose(
		goal.pose.position.x,
		goal.pose.position.y,
		tf2::getYaw(goal.pose.orientation)));

	// create start to tree
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
		Pose pose = this->goal_sample_distribution(this->rng) < EPSILON ? goal_node->pose : this->sample_random_pose();
		double edge_end[] = {pose.x, pose.y, pose.yaw};

		// get nearest node in the tree
		auto nearest = this->get_nearest_node(pose);
		double edge_begin[] = {nearest->pose.x, nearest->pose.y, nearest->pose.yaw};

		// tree edge is the shortest dubins path between the two poses
		DubinsPath edge;		
		dubins_shortest_path(&edge, edge_begin, edge_end, TURNING_RADIUS);

		// saturate
		if (dubins_path_length(&edge) > RRT_STEP_SIZE)
		{
			pose = this->steer(edge, RRT_STEP_SIZE);
			dubins_extract_subpath(&edge, RRT_STEP_SIZE, &edge);
		}

		// TODO error on dubins return != 0

		// move on to next ite if edge is invalid
		if (this->is_path_in_collision(edge))
		{
			continue;
		}
		
		// add node to the tree
		auto node = this->add_node(pose, nearest, edge);

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


bool RRTPlanner::is_path_in_collision(DubinsPath& path) const
{
	double sample[3];
	Pose pose;
    double t = 0;
    double length = dubins_path_length(&path);

    while(t <  length)
	{
        dubins_path_sample(&path, t, sample);
		pose.x = sample[0];
		pose.y = sample[1];
		pose.yaw = sample[2];

        if (this->is_pose_in_collision(pose))
		{
			return true;
		}
		
        t += DUBINS_COL_STEP_SIZE;
    }

	// last pose in the path
	dubins_path_sample(&path, length, sample);
	pose.x = sample[0];
	pose.y = sample[1];
	pose.yaw = sample[2];

	return this->is_pose_in_collision(pose);
}


Pose RRTPlanner::sample_random_pose()
{
	Pose pose;
	do
	{
		pose.x = this->pose_distribution[0](this->rng);
		pose.y = this->pose_distribution[1](this->rng);
		pose.yaw = this->pose_distribution[2](this->rng);
	}
	while (this->is_pose_in_collision(pose));
	
	return pose;
}


Pose RRTPlanner::steer(DubinsPath& path, const double t) const
{	
	double sample[3];
	dubins_path_sample(&path, t, sample);

	return Pose(sample[0], sample[1], sample[2]);
}


std::shared_ptr<Node> RRTPlanner::get_nearest_node(const Pose& pose) const
{
	std::shared_ptr<Node> nearest = nullptr;
	double min_dist = std::numeric_limits<double>::max();

	for (const auto& elem : this->tree)
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


bool RRTPlanner::reconnect_node(std::shared_ptr<Node> node, std::shared_ptr<Node> parent)
{
	double edge_begin[] = {parent->pose.x, parent->pose.y, parent->pose.yaw};
	double edge_end[] = {node->pose.x, node->pose.y, node->pose.yaw};

	// create edge between the two nodes
	DubinsPath edge;		
	dubins_shortest_path(&edge, edge_begin, edge_end, TURNING_RADIUS);
	double cost = parent->cost + dubins_path_length(&edge);

	// no need to reconnect if it's more expensive or there are obstacles
	if (cost >= node->cost || this->is_path_in_collision(edge))
	{
		return false;
	}

	// reconnect the node
	node->parent = parent;
	node->cost = cost;
	node->edge = edge;
	this->tree.push_back(node);

	return true;
}


std::shared_ptr<Node> RRTPlanner::add_node(const Pose& pose, std::shared_ptr<Node> parent, DubinsPath& edge)
{
	// add new node to the tree
	auto node = std::make_shared<Node>(pose);
	node->parent = parent;
	node->cost = parent->cost + dubins_path_length(&edge);
	node->edge = edge;
	this->tree.push_back(node);

	return node;
}

void RRTPlanner::publish_path(const std::vector<geometry_msgs::PoseStamped>& path) const
{
	nav_msgs::Path msg;
	msg.header.frame_id = this->costmap_ros->getGlobalFrameID();
	msg.header.stamp = ros::Time::now();
	msg.poses = path;

	this->plan_pub.publish(msg);
}

bool RRTPlanner::retrace_path(std::shared_ptr<Node> node, std::vector<geometry_msgs::PoseStamped>& path)
{	
	if (node == nullptr || node->parent == nullptr)
	{
		return false;
	}
	
	while (node->parent != nullptr)
	{
		double sample[3];
		double t = dubins_path_length(&node->edge);
		while(t > 0)
		{
			dubins_path_sample(&node->edge, t, sample);

			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = this->costmap_ros->getGlobalFrameID();
			pose.pose.position.x = sample[0];
			pose.pose.position.y = sample[1];
			tf2::Quaternion quat;
			quat.setRPY(0, 0, sample[3]);
			pose.pose.orientation = tf2::toMsg(quat);
			path.push_back(pose);

			t -= DUBINS_PUB_STEP_SIZE;
		}

		// t never reaches 0, so we add the parent here
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = this->costmap_ros->getGlobalFrameID();
		pose.pose.position.x = node->parent->pose.x;
		pose.pose.position.y = node->parent->pose.y;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, node->parent->pose.yaw);
		pose.pose.orientation = tf2::toMsg(quat);
		path.push_back(pose);

		node = node->parent;
	}

	std::reverse(std::begin(path), std::end(path));

	return true;
}


void RRTPlanner::clear_markers() const
{
	visualization_msgs::Marker msg;
	msg.header.frame_id = this->costmap_ros->getGlobalFrameID();
	msg.header.stamp = ros::Time::now();
	msg.ns = "rapidly_exploring_random_tree";
	msg.action = visualization_msgs::Marker::DELETEALL;
	msg.type = visualization_msgs::Marker::LINE_STRIP;
	this->tree_pub.publish(msg);
}


void RRTPlanner::publish_tree_cb(const ros::TimerEvent& event)
{	
	// planning stopped
	if (!this->is_planning.load())
	{
		// last tree not published yet
		if (!this->is_last_tree_published.load())
		{
			this->is_last_tree_published.store(true);
		}
		else
		{
			return;
		}
	}
	else
	{
		this->is_last_tree_published.store(false);
	}
	
	
	std::vector<std::shared_ptr<Node>> nodes;
	
	// thread safe deep copy
	{
		std::lock_guard<std::mutex> lock(this->tree_mutex);
		nodes = this->tree;
	}

	// skip tree root
	for (size_t i = 1; i  < nodes.size(); ++i)
	{
		const std::shared_ptr<Node>& node = nodes[i];

		visualization_msgs::Marker msg;
		msg.header.frame_id = this->costmap_ros->getGlobalFrameID();
		msg.header.stamp = event.current_real;
		msg.ns = "rapidly_exploring_random_tree";
		msg.id = i;
		msg.action = visualization_msgs::Marker::ADD;
		msg.type = visualization_msgs::Marker::LINE_STRIP;
		msg.scale.x = 0.03;
		msg.color.g = 1;
		msg.color.a = 0.5;
		msg.pose.orientation.w = 1;

		double sample[3];
		double t = dubins_path_length(&node->edge);
		while(t > 0)
		{
			dubins_path_sample(&node->edge, t, sample);

			geometry_msgs::Point p;
			p.x = sample[0];
			p.y = sample[1];
			msg.points.push_back(p);

			t -= DUBINS_PUB_STEP_SIZE;
		}

		// t never reaches 0, so we add the parent here
		geometry_msgs::Point p;
		p.x = node->parent->pose.x;
		p.y = node->parent->pose.y;
		msg.points.push_back(p);

		this->tree_pub.publish(msg);
	}
}

}
