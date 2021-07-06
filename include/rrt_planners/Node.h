#pragma once

#include "rrt_planners/dubins.h"
#include "rrt_planners/Pose.h"
#include <unordered_map>

namespace rrt_planners
{
	struct Node
	{
		Node()
		: Node(Pose())
		{};
		
		Node(const Pose& pose)
		: pose(pose), cost(std::numeric_limits<double>::max()), lmc(std::numeric_limits<double>::max())
		{};

		/** Pose of the node. */
		const Pose pose;
		
		/** Cost to reach this node from the root node. */
		double cost;

		/** Parent of this node. */
		std::shared_ptr<Node> parent;

		/** Edge from this node to the parent. */
		std::shared_ptr<DubinsPath> edge;


		/* --- RRTX ONLY --- */

		/** Look-ahead estimate of cost-to-goal. */
		double lmc;

		/** Children of this node. */
		std::vector<std::shared_ptr<Node>> children;

		/** Original PRM*-like in-neighbours. */
		std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<DubinsPath>> original_in_neighbours;

		/** Original PRM*-like out-neighbours. */
		std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<DubinsPath>> original_out_neighbours;

		/** Running in-neighbours. */
		std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<DubinsPath>> running_in_neighbours;

		/** Running out-neighbours. */
		std::unordered_map<std::shared_ptr<Node>, std::shared_ptr<DubinsPath>> running_out_neighbours;	
	};
}
