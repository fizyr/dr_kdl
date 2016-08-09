#include "eigen.hpp"

#include <dr_eigen/eigen.hpp>

#include <gtest/gtest.h>
#include <string>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

std::string toString(Eigen::Vector3d const & q) {
	return "[" + std::to_string(q.w())  + ", " + std::to_string(q.x()) + ", " + std::to_string(q.y()) + ", " + std::to_string(q.z()) + "]";
}

std::string toString(Eigen::Quaterniond const & q) {
	return "[" + std::to_string(q.w())  + ", " + std::to_string(q.x()) + ", " + std::to_string(q.y()) + ", " + std::to_string(q.z()) + "]";
}

std::string toString(Eigen::Matrix3d const & q) {
	return toString(Eigen::Quaterniond(q));
}

std::string toString(Eigen::Isometry3d const & q) {
	return "{" + toString(Eigen::Vector3d{q.translation()}) + " * " + toString(q.rotation()) + "}";
}

std::string toString(KDL::Vector const & v) {
	return "[" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ", " + std::to_string(v.z()) + "]";
}

std::string toString(KDL::Rotation const & q) {
	double w, x, y, z;
	q.GetQuaternion(x, y, z, w);
	return "[" + std::to_string(w)  + ", " + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "]";
}

std::string toString(KDL::Frame const & q) {
	return "{" + toString(q.p) + " * " + toString(q.M) + "}";
}

TEST(EigenConversions, vector) {
	ASSERT_EQ(toEigen(KDL::Vector(0, 1, 2)), Eigen::Vector3d(0, 1, 2));
	ASSERT_NE(toEigen(KDL::Vector(0, 1, 2)), Eigen::Vector3d(0, 1, 3));
	ASSERT_EQ(toKdlVector(Eigen::Vector3d(0, 1, 2)), KDL::Vector(0, 1, 2));
	ASSERT_NE(toKdlVector(Eigen::Vector3d(0, 1, 2)), KDL::Vector(0, 1, 3));
}

testing::AssertionResult testVector(Eigen::Vector3d const & eigen, KDL::Vector const & kdl) {
	Eigen::Vector3d eigen_converted = toEigen(kdl);
	KDL::Vector     kdl_converted   = toKdlVector(eigen);
	if (!eigen_converted.isApprox(eigen)) return testing::AssertionFailure() << "toEigen(kdl) " << toString(eigen_converted) << " does not match eigen " << toString(eigen);
	if (!KDL::Equal(kdl_converted, kdl))  return testing::AssertionFailure() << "toKDL(eigen) " << toString(kdl_converted)   << " does not match kdl " << toString(kdl);
	return testing::AssertionSuccess() << "eigen and kdl values convert to eachother";
}

testing::AssertionResult testRotation(Eigen::Quaterniond const & eigen, KDL::Rotation const & kdl) {
	Eigen::Quaterniond eigen_converted = toEigen(kdl);
	KDL::Rotation      kdl_converted   = toKdlRotation(eigen);
	if (!eigen_converted.isApprox(eigen)) return testing::AssertionFailure() << "toEigen(kdl) " << toString(eigen_converted) << " does not match eigen " << toString(eigen);
	if (!KDL::Equal(kdl_converted, kdl))  return testing::AssertionFailure() << "toKDL(eigen) " << toString(kdl_converted)   << " does not match kdl " << toString(kdl);
	return testing::AssertionSuccess() << "eigen and kdl values convert to eachother";
}

testing::AssertionResult testRotation(Eigen::AngleAxisd const & eigen, KDL::Rotation const & kdl) {
	return testRotation(Eigen::Quaterniond(eigen), kdl);
}

testing::AssertionResult testRotation(Eigen::Matrix3d const & eigen, KDL::Rotation const & kdl) {
	Eigen::Matrix3d eigen_converted = toEigenMatrix(kdl);
	KDL::Rotation   kdl_converted   = toKdlRotation(eigen);
	if (!eigen_converted.isApprox(eigen)) return testing::AssertionFailure() << "toEigen(kdl) " << toString(eigen_converted) << " does not match eigen " << toString(eigen);
	if (!KDL::Equal(kdl_converted, kdl))  return testing::AssertionFailure() << "toKDL(eigen) " << toString(kdl_converted)   << " does not match kdl " << toString(kdl);
	return testing::AssertionSuccess() << "eigen and kdl values convert to eachother";
}

testing::AssertionResult testIsometry(Eigen::Isometry3d const & eigen, KDL::Frame const & kdl) {
	Eigen::Isometry3d eigen_converted = toEigen(kdl);
	KDL::Frame        kdl_converted   = toKdlFrame(eigen);
	if (!eigen_converted.isApprox(eigen)) return testing::AssertionFailure() << "toEigen(kdl) " << toString(eigen_converted) << " does not match eigen " << toString(eigen);
	if (!KDL::Equal(kdl_converted, kdl))  return testing::AssertionFailure() << "toKDL(eigen) " << toString(kdl_converted)   << " does not match kdl " << toString(kdl);
	return testing::AssertionSuccess() << "eigen and kdl values convert to eachother";
}

TEST(EigenConversions, quaternion) {
	ASSERT_TRUE(testRotation(Eigen::Quaterniond::Identity(), KDL::Rotation::Identity()));
	ASSERT_TRUE(testRotation(rotateX(1), KDL::Rotation::RotX(1)));
	ASSERT_TRUE(testRotation(rotateY(2), KDL::Rotation::RotY(2)));
	ASSERT_TRUE(testRotation(rotateZ(3), KDL::Rotation::RotZ(3)));

	// TODO: Why does this one fail?
	//ASSERT_TRUE(testRotation(rotateX(1) * rotateY(2) * rotateZ(3), KDL::Rotation::RotX(1) * KDL::Rotation::RotY(2) * KDL::Rotation::RotZ(3)));
}

TEST(EigenConversions, matrix) {
	ASSERT_TRUE(testRotation(Eigen::Matrix3d::Identity(), KDL::Rotation::Identity()));
	ASSERT_TRUE(testRotation(rotateX(1).matrix(), KDL::Rotation::RotX(1)));
	ASSERT_TRUE(testRotation(rotateY(2).matrix(), KDL::Rotation::RotY(2)));
	ASSERT_TRUE(testRotation(rotateZ(3).matrix(), KDL::Rotation::RotZ(3)));
	ASSERT_TRUE(testRotation((rotateX(1) * rotateY(2) * rotateZ(3)).matrix(), KDL::Rotation::RotX(1) * KDL::Rotation::RotY(2) * KDL::Rotation::RotZ(3)));
}

TEST(EigenConversions, isometry) {
	ASSERT_TRUE(testIsometry(Eigen::Isometry3d::Identity(), KDL::Frame::Identity()));
	ASSERT_TRUE(testIsometry(translate(0, 1, 2) * rotateZ(3), KDL::Frame(KDL::Rotation::RotZ(3), KDL::Vector(0, 1, 2))));
	ASSERT_TRUE(testIsometry(translate(0, 1, 2) * rotateX(1) * rotateY(2) * rotateZ(3), KDL::Frame(KDL::Rotation::RotX(1) * KDL::Rotation::RotY(2) * KDL::Rotation::RotZ(3), KDL::Vector(0, 1, 2))));
}

}
