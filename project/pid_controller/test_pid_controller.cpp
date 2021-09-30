#include <gtest/gtest.h>

#include "pid_controller.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

TEST(PidControllerTest, P_ConstantError) {
  	PID controller = PID();
  	controller.Init(/*Kp*/ 1, /*Ki*/ 0, /*Kd*/ 0, /*output_lim_max*/ 10.0, /*output_lim_min*/ -10.0);
  	controller.UpdateDeltaTime(1);
  	controller.UpdateError(1);
  	double error = controller.TotalError();
  	EXPECT_FLOAT_EQ(-1.0, error);
  
  	controller.UpdateDeltaTime(1);
  	controller.UpdateError(1);
  	error = controller.TotalError();
  	EXPECT_FLOAT_EQ(-1.0, error);
}

TEST(PidControllerTest, I_ConstantError) {
  	PID controller = PID();
  	controller.Init(/*Kp*/ 0, /*Ki*/ 1, /*Kd*/ 0, /*output_lim_max*/ 10.0, /*output_lim_min*/ -10.0);
  	controller.UpdateDeltaTime(1);
  	controller.UpdateError(1);
  	double error = controller.TotalError();
  	EXPECT_FLOAT_EQ(-1.0, error);
  
  	controller.UpdateDeltaTime(1);
  	controller.UpdateError(1);
  	error = controller.TotalError();
  	EXPECT_FLOAT_EQ(-2.0, error);
}

TEST(PidControllerTest, D_ConstantError) {
  	PID controller = PID();
  	controller.Init(/*Kp*/ 0, /*Ki*/ 0, /*Kd*/ 1, /*output_lim_max*/ 10.0, /*output_lim_min*/ -10.0);
  	controller.UpdateDeltaTime(1);
  	controller.UpdateError(1);
  	double error = controller.TotalError();
  	EXPECT_FLOAT_EQ(-0.0, error);
  
  	controller.UpdateDeltaTime(1);
  	controller.UpdateError(0);
  	error = controller.TotalError();
  	EXPECT_FLOAT_EQ(1.0, error);
}
