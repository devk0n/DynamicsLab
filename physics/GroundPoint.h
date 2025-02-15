#ifndef GROUNDPOINT_H
#define GROUNDPOINT_H

#include <Eigen/Dense>


class GroundPoint {
public:
	explicit GroundPoint(Eigen::Vector3d position);

	[[nodiscard]] Eigen::Vector3d getPosition() const;

private:
	const Eigen::Vector3d m_position;
};


#endif // GROUNDPOINT_H
