#include "dr_kdl.hpp"

#include <dr_eigen/eigen.hpp>
#include <dr_util/geometry.hpp>

#include <gtest/gtest.h>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(KdlTree, pose) {
	KdlTree kdl;
	kdl.addSegment(KDL::Segment("arm", KDL::Joint("joint", KDL::Joint::RotZ), KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(1, 0, 0))), "root");

	ASSERT_TRUE(kdl.pose("root", "arm", {{"joint", 0.0 * pi()}}).isApprox(translate(1, 0, 0)  * rotateZ(0.0 * pi())));
	ASSERT_TRUE(kdl.pose("root", "arm", {{"joint", 0.5 * pi()}}).isApprox(translate(0, 1, 0)  * rotateZ(0.5 * pi())));
	ASSERT_TRUE(kdl.pose("root", "arm", {{"joint", 1.0 * pi()}}).isApprox(translate(-1, 0, 0) * rotateZ(1.0 * pi())));
	ASSERT_TRUE(kdl.pose("root", "arm", {{"joint", 1.5 * pi()}}).isApprox(translate(0, -1, 0) * rotateZ(1.5 * pi())));
	ASSERT_TRUE(kdl.pose("root", "arm", {{"joint", 2.0 * pi()}}).isApprox(translate(1, 0, 0)  * rotateZ(0.0 * pi())));
}

TEST(KdlTree, transform) {
	KdlTree kdl;
	kdl.addSegment(KDL::Segment("arm", KDL::Joint("joint", KDL::Joint::RotZ), KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(1, 0, 0))), "root");

	ASSERT_TRUE(kdl.transform("arm", "root", {{"joint", 0.0 * pi()}}).isApprox(translate(1, 0, 0)  * rotateZ(0.0 * pi())));
	ASSERT_TRUE(kdl.transform("arm", "root", {{"joint", 0.5 * pi()}}).isApprox(translate(0, 1, 0)  * rotateZ(0.5 * pi())));
	ASSERT_TRUE(kdl.transform("arm", "root", {{"joint", 1.0 * pi()}}).isApprox(translate(-1, 0, 0) * rotateZ(1.0 * pi())));
	ASSERT_TRUE(kdl.transform("arm", "root", {{"joint", 1.5 * pi()}}).isApprox(translate(0, -1, 0) * rotateZ(1.5 * pi())));
	ASSERT_TRUE(kdl.transform("arm", "root", {{"joint", 2.0 * pi()}}).isApprox(translate(1, 0, 0)  * rotateZ(0.0 * pi())));
}

}
