#pragma once

#include <utility>

#include <kdl/tree.hpp>
#include <dr_util/eigen.hpp>

namespace dr {

/// Create an Eigen transform from a KDL Frame.
inline Eigen::Isometry3d toEigen(KDL::Frame const & frame) {
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
	for (int j = 0; j < 4; ++j) {
		result(3, j) = j == 3;
	}

	return result;
}

/// Get a the transform from the base to the end of a chain.
/**
 * Throws if a non-fixed joint is encountered in the chain.
 * \return The transform from the base of the chain to the end.
 */
Eigen::Isometry3d getTransform(
	KDL::Chain const & chain ///< The chain.
);


/// KDL tree wrapper.
class KdlTree : public KDL::Tree {
public:
	KdlTree() {}
	KdlTree(KDL::Tree const &  tree) : KDL::Tree{tree} {}
	KdlTree(KDL::Tree const && tree) : KDL::Tree{std::move(tree)} {}

	static KdlTree fromString(std::string const & urdf);
	static KdlTree fromParameter(std::string const & parameter);
	static KdlTree fromFile(std::string const & filename);

	/// Get a transform from one frame to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint.
	 */
	Eigen::Isometry3d transform(std::string const & source, std::string const & target) const;
};

}
