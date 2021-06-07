#pragma once

namespace rrt_planners
{
	struct Pose
	{
		Pose()
		: Pose(0, 0, 0)
		{};

		Pose(const double x, const double y, const double yaw)
		: x(x), y(y), yaw(yaw)
		{};

		/**
		 * Computes the distance to the given pose.
		 * 
		 * @param pose	Pose to calculate distance to.
		 * @param c		Weight for the heading distance.
		 * 
		 * @return		The computed distance.
		 */
		double distance_to(const Pose& pose, double c = 1.0) const
		{
			return sqrt(
				c * pow(this->yaw - pose.yaw, 2) +
				pow(this->x - pose.x, 2) +
				pow(this->y - pose.y, 2));
		}

		/** The X coordinate in meters. */
		double x;

		/** The Y coordinate in meters. */
		double y;

		/** The Yaw angle in radians. */
		double yaw;
	};
}
