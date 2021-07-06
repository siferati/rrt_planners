#pragma once

#include "rrt_planners/RRTStarPlanner.h"

namespace rrt_planners
{

class RRTXPlanner : public RRTStarPlanner
{
public:

	// TODO dynamic reconfigure
	static constexpr double EPSILON = 0.5;

	virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

	virtual bool makePlan(
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& path) override;

	/**
	 * Creates a new node with the given pose and adds it to the tree.
	 * 
	 * @param pose	Pose of the new node.
	 * 
	 * @return 		The new node.
	 */ 
	std::shared_ptr<Node> add_node(const Pose& pose);

	/**
	 * Rewires the given node's in-neighbours
	 * to use it as parent if it's better.
	 * 
	 * @param node Node to evaluate.
	 */ 
	void rewire_neighbours(std::shared_ptr<Node> node);


	/**
	 * Culls the running neighbours from the given node,
	 * to allow only edges that are shorter than the shrinking ball radius
	 * or edges that are part of the tree.
	 */ 
	void cull_neighbours(std::shared_ptr<Node> node);
};

}
