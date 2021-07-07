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
	 * Culls the running neighbours from the given node,
	 * to allow only edges that are shorter than the shrinking ball radius
	 * or edges that are part of the tree.
	 */ 
	void cull_neighbours(std::shared_ptr<Node> node);

	/**
	 * Rewires the given node's in-neighbours
	 * to use it as parent if it's better.
	 * 
	 * @param node Node to evaluate.
	 */ 
	void rewire_neighbours(std::shared_ptr<Node> node);

	/**
	 * Propagates cost-to-goal information
	 * through a rewiring cascade and makes nodes consistent.
	 */ 
	void reduce_inconsistency();

	/**
	 * Update node's lmc based on its out-neighbours.
	 * 
	 * @param node Node to update.
	 */ 
	void update_lmc(std::shared_ptr<Node> node);

protected:

	/** Priority queue of nodes that are not e-consistent. */
	std::set<
		std::shared_ptr<Node>,
		std::function<bool(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2)>
	> inconsistent_nodes;
};

}
