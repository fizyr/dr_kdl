#pragma once
#include <map>
#include <string>

#include <Eigen/Geometry>
#include <kdl/tree.hpp>
#include <sensor_msgs/JointState.h>

namespace dr {


/// Get a the pose of the end frame relative to the start frame of a chain.
/**
 * Throws if a non-fixed joint is encountered in the chain.
 * \return The transform from the base of the chain to the end.
 */
Eigen::Isometry3d getPose(
	KDL::Chain const & chain ///< The chain.
);


/// Get a the pose of the end frame relative to the start frame of a chain.
/**
 * Non-fixed joints are looked up in a map.
 * Throws if a joint is not found.
 * \return The transform from the base of the chain to the end.
 */
Eigen::Isometry3d getPose(
	KDL::Chain const & chain,                    ///< The chain.
	std::map<std::string, double> const & joints ///< The map of joint positions to use for non fixed joints in the chain.
);


/// Get a the pose of the end frame relative to the start frame of a chain.
/**
 * Non-fixed joints are looked up in a pair of vectors.
 * Throws if a joint is not found.
 * \return The transform from the base of the chain to the end.
 */
Eigen::Isometry3d getPose(
	KDL::Chain const & chain,                     ///< The chain.
	std::vector<std::string> const & joint_names, ///< The names of the joints in same order as the joint position vector.
	std::vector<double> const & joint_positions   ///< The positions of the joints in the same order as the joint name vector.
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

	/// Get a KDL chain between two segments.
	KDL::Chain getChain(
		std::string const & start, /// The start segment.
		std::string const & end    /// The end segment.
	) const;

	/// Get a pose of one frame relative to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint.
	 */
	Eigen::Isometry3d pose(
		std::string const & source, ///< The source frame.
		std::string const & target  ///< The target frame.
	) const {
		return getPose(getChain(source, target));
	}

	/// Get a pose of one frame relative to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint for which no joint position is given.
	 */
	Eigen::Isometry3d pose(
		std::string const & source,                   ///< The source frame.
		std::string const & target,                   ///< The target frame.
		std::map<std::string, double> const & joints  ///< The map holding joint positions.
	) const {
		return getPose(getChain(source, target), joints);
	}

	/// Get a pose of one frame relative to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint for which no joint position is given.
	 */
	Eigen::Isometry3d pose(
		std::string const & source,                   ///< The source frame.
		std::string const & target,                   ///< The target frame.
		std::vector<std::string> const & joint_names, ///< The names of the joints in same order as the joint position vector.
		std::vector<double> const & joint_positions   ///< The positions of the joints in the same order as the joint name vector.
	) const {
		return getPose(getChain(source, target), joint_names, joint_positions);
	}

	/// Get a pose of one frame relative to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint for which no joint position is given.
	 */
	Eigen::Isometry3d pose(
		std::string const & source,                   ///< The source frame.
		std::string const & target,                   ///< The target frame.
		sensor_msgs::JointState const & joints        ///< The joint positions.
	) const {
		return getPose(getChain(source, target), joints.name, joints.position);
	}

	/// Get a transformation that transforms coordinates from one frame to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint.
	 */
	Eigen::Isometry3d transform(
		std::string const & source, ///< The source frame.
		std::string const & target  ///< The target frame.
	) const {
		return pose(target, source);
	}

	/// Get a transformation that transforms coordinates from one frame to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint for which no joint position is given.
	 */
	Eigen::Isometry3d transform(
		std::string const & source,                   ///< The source frame.
		std::string const & target,                   ///< The target frame.
		std::map<std::string, double> const & joints  ///< The map holding joint positions.
	) const {
		return pose(target, source, joints);
	}

	/// Get a transformation that transforms coordinates from one frame to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint for which no joint position is given.
	 */
	Eigen::Isometry3d transform(
		std::string const & source,                   ///< The source frame.
		std::string const & target,                   ///< The target frame.
		std::vector<std::string> const & joint_names, ///< The names of the joints in same order as the joint position vector.
		std::vector<double> const & joint_positions   ///< The positions of the joints in the same order as the joint name vector.
	) const {
		return pose(target, source, joint_names, joint_positions);
	}

	/// Get a transformation that transforms coordinates from one frame to another.
	/**
	 * Throws if there is no chain between the frames or the chain contains a non-fixed joint for which no joint position is given.
	 */
	Eigen::Isometry3d transform(
		std::string const & source,                   ///< The source frame.
		std::string const & target,                   ///< The target frame.
		sensor_msgs::JointState const & joints        ///< The joint positions.
	) const {
		return pose(target, source, joints);
	}

};

}
