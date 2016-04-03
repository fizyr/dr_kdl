#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dr {

/// Convert a KDL vector to an Eigen vector.
Eigen::Vector3d toEigen(KDL::Vector const & input) {
	return Eigen::Vector3d{input[0], input[1], input[2]};
}

/// Convert an Eigen vector to a KDL vector.
KDL::Vector toKdlVector(Eigen::Vector3d const & input) {
	return KDL::Vector{input.x(), input.y(), input.z()};
}

/// Convert a KDL rotation to an Eigen quaternion.
Eigen::Quaterniond toEigen(KDL::Rotation const & input) {
	Eigen::Quaterniond result;
	input.GetQuaternion(result.x(), result.y(), result.z(), result.w());
	return result;
}

/// Convert a KDL rotation to an Eigen rotation matrix.
Eigen::Matrix3d toEigenMatrix(KDL::Rotation const & input) {
	return (Eigen::Matrix3d{} <<
		input(0, 0),  input(0, 1), input(0, 2),
		input(1, 0),  input(1, 1), input(1, 2),
		input(2, 0),  input(2, 1), input(2, 2)
	).finished();
}

/// Convert an Eigen quaternion to a KDL rotation.
KDL::Rotation toKdlRotation(Eigen::Quaterniond const & input) {
	return KDL::Rotation::Quaternion(input.x(), input.y(), input.z(), input.w());
}

/// Convert an Eigen rotation matrix to a KDL rotation.
KDL::Rotation toKdlRotation(Eigen::Matrix3d const & input) {
	KDL::Rotation result;
	for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
		result(i, j) = input(i, j);
	}
	return result;
}

/// Convert a KDL frame to an Eigen isometry.
Eigen::Isometry3d toEigen(KDL::Frame const & input) {
	return Eigen::Translation3d(toEigen(input.p)) * toEigen(input.M);
}

/// Convert an Eigen isometry to a KDL frame.
KDL::Frame toKdlFrame(Eigen::Isometry3d const & input) {
	return {toKdlRotation(input.rotation()), toKdlVector(Eigen::Vector3d{input.translation()})};
}

}
