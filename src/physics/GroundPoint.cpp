#include "GroundPoint.h"

#include <utility>

GroundPoint::GroundPoint(Eigen::Vector3d position) : m_position(std::move(position)) {
}

Eigen::Vector3d GroundPoint::getPosition() const {
	return m_position;
}

