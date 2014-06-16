#include <algorithm>
#include <stdexcept>

#include <kdl_parser/kdl_parser.hpp>

#include "dr_kdl.hpp"

namespace dr {

namespace {
	/// Create an Eigen transform from a KDL Frame.
	Eigen::Isometry3d toEigen(KDL::Frame const & frame) {
		Eigen::Isometry3d result;

		// Rotation.
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				result(i, j) = frame.M(i, j);
			}
		}

		// Translation.
		for (int i = 0; i < 3; ++i) {
			result(i, 3) = frame.p(i);
		}

		// Homogenous bit.
		result(3, 0) = 0;
		result(3, 1) = 0;
		result(3, 2) = 0;
		result(3, 3) = 1;

		return result;
	}
}

/// Get a the transform from the base to the tip of a chain.
Eigen::Isometry3d getTransform(KDL::Chain const & chain) {
	KDL::Frame transform = KDL::Frame::Identity();
	for (auto const & segment : chain.segments) {
		// Make sure we're only dealing with fixed joints.
		if (segment.getJoint().getType() != KDL::Joint::JointType::None) {
			throw std::runtime_error("Non-fixed joint `" + segment.getName() + "' found in chain, but no joint positions are given.");
		}
		transform = transform * segment.pose(0);
	}
	return toEigen(transform);
}

/// Get a the transform from the base to the tip of a chain.
Eigen::Isometry3d getTransform(KDL::Chain const & chain, std::map<std::string, double> const & joints) {
	KDL::Frame transform = KDL::Frame::Identity();

	for (auto const & segment : chain.segments) {
		// Fixed joints.
		if (segment.getJoint().getType() == KDL::Joint::JointType::None) {
			transform = transform * segment.pose(0);
		
		// Look up non-fixed joints in map.
		} else {
			auto joint = joints.find(segment.getJoint().getName());
			if (joint == joints.end()) throw std::runtime_error("Joint `" + segment.getName() + "' not found in joint map.");
			transform = transform * segment.pose(joint->second);
		}
	}

	return toEigen(transform);
}

/// Get a the transform from the base to the tip of a chain.
Eigen::Isometry3d getTransform(KDL::Chain const & chain, std::vector<std::string> const & joint_names, std::vector<double> const & joint_positions) {
	KDL::Frame transform = KDL::Frame::Identity();

	for (auto const & segment : chain.segments) {
		// Fixed joints.
		if (segment.getJoint().getType() == KDL::Joint::JointType::None) {
			transform = transform * segment.pose(0);
		
		// Look up non-fixed joints in map.
		} else {
			auto index = std::find(joint_names.begin(), joint_names.end(), segment.getJoint().getName());
			if (index == joint_names.end()) throw std::runtime_error("Joint `" + segment.getName() + "' not found in joint map.");
			transform = transform * segment.pose(joint_positions[std::distance(joint_names.begin(), index)]);
		}
	}

	return toEigen(transform);
}

KDL::Chain KdlTree::getChain(std::string const & start, std::string const & end) const {
	KDL::Chain chain;
	if (KDL::Tree::getChain(start, end, chain)) {
		return chain;
	} else {
		throw std::runtime_error("No chain found from frame `" + start + "' to frame `" + end + "'.");
	}
}

KdlTree KdlTree::fromParameter(std::string const & parameter) {
	KDL::Tree kdl;
	kdl_parser::treeFromParam(parameter, kdl);
	return kdl;
}

KdlTree KdlTree::fromString(std::string const & urdf) {
	KDL::Tree kdl;
	kdl_parser::treeFromString(urdf, kdl);
	return kdl;
}

KdlTree KdlTree::fromFile(std::string const & filename) {
	KDL::Tree kdl;
	kdl_parser::treeFromFile(filename, kdl);
	return kdl;
}

}
