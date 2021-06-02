#pragma once

#include "rrt_planners/dubins.h"

namespace rrt_planners
{
	struct Node
	{
		Node(double x, double y, double w)
		: x(x), y(y), w(w), cost(std::numeric_limits<double>::max())
		{};

		/** Pose of the node. */
		double x, y, w;

		/** Parent of this node. */
		std::shared_ptr<const Node> parent;

		/** Edge from this node to the parent. */
		std::unique_ptr<const DubinsPath> edge;	

		/** Cost to reach this node from the root node. */
		double cost;
	};
}
