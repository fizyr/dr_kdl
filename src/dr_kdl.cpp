#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include "dr_kdl.hpp"

namespace dr {

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


Eigen::Isometry3d KdlTree::transform(std::string const & source, std::string const & target) const {
	KDL::Chain chain;
	if (getChain(source, target, chain)) {
		return getTransform(chain);
	} else {
		throw std::runtime_error("No chain found from frame `" + source + "' to frame `" + target + "'.");
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
