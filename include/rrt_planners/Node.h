#pragma once

#include "rrt_planners/dubins.h"
#include "rrt_planners/Pose.h"

namespace rrt_planners
{
	struct Node
	{
		Node()
		: Node(Pose())
		{};
		
		Node(const Pose& pose)
		: pose(pose), cost(std::numeric_limits<double>::max())
		{};

		/** Pose of the node. */
		const Pose pose;
		
		/** Cost to reach this node from the root node. */
		double cost;

		/** Parent of this node. */
		std::shared_ptr<Node> parent;

		/** Edge from this node to the parent. */
		DubinsPath edge;	
	};
}
