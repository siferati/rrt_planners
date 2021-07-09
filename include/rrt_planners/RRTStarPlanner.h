#pragma once

#include "rrt_planners/RRTPlanner.h"

namespace rrt_planners
{

class RRTStarPlanner : public RRTPlanner
{
public:
	// TODO actually shrink it
	static constexpr double SHRINKING_RADIUS = 2.0;

	virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

	virtual bool makePlan(
		const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& path) override;

	/**
	 * Searches the tree for all the nodes within the given radius of the given pose.
	 * 
	 * @param pose		Pose to evaluate.
	 * @param radius	Radius to search.
	 * 
	 * @return		Nodes that are less than radius distance away from pose.
	 */ 
	std::vector<std::shared_ptr<Node>> get_nodes_in_radius(const Pose& pose, const double radius) const;
};

}
