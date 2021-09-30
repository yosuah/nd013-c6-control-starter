#include <gtest/gtest.h>

#include "utils.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

TEST(UtilsTest, SteeringErrorHorizontalLaneFromCenter) {
	double error = utils::get_steer_error(0, 0, 0, 10, 10);
  	EXPECT_FLOAT_EQ(10.0, error);
}

TEST(UtilsTest, SteeringErrorHorizontalLaneFromOffset) {
  	double error = utils::get_steer_error(10, 10, 0, 20, 20);
  	EXPECT_FLOAT_EQ(10.0, error);
}

TEST(UtilsTest, SteeringErrorAtAngleFromOffset) {
  	double error = utils::get_steer_error(10, 10, M_PI / 4, 0, 20);
  	EXPECT_FLOAT_EQ(std::sqrt(200), error);
}
