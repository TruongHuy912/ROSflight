#include <gtest/gtest.h>

#include <controller/controller_state_machine.hpp>

#include <cmath>
#include <memory>

namespace
{

class TestController : public roscopter::ControllerStateMachine
{
public:
  rosflight_msgs::msg::Command step(double dt)
  {
    return manage_state(input_command_, status_, dt);
  }

  void start_takeoff()
  {
    input_command_.cmd_valid = true;
    status_.armed = true;
    step(0.1);
    ASSERT_EQ(get_controller_state(), TAKEOFF);
  }

  void set_estimate(double position_d, double velocity_d, double time = 0.0)
  {
    xhat_.p_d = position_d;
    xhat_.v_x = 0.0;
    xhat_.v_y = 0.0;
    xhat_.v_z = velocity_d;
    xhat_.quat.w = 1.0;
    xhat_.quat.x = 0.0;
    xhat_.quat.y = 0.0;
    xhat_.quat.z = 0.0;
    xhat_.header.stamp.sec = static_cast<int32_t>(time);
    xhat_.header.stamp.nanosec =
      static_cast<uint32_t>((time - std::floor(time)) * 1e9);
  }

  void configure_smooth_takeoff(
    double max_accel = 0.1,
    double position_tolerance = 0.1,
    double velocity_tolerance = 0.1,
    double settle_time = 1.0)
  {
    set_parameters({
      rclcpp::Parameter("smooth_takeoff_enabled", true),
      rclcpp::Parameter("takeoff_d_pos", -2.0),
      rclcpp::Parameter("takeoff_d_vel", -0.5),
      rclcpp::Parameter("takeoff_max_accel", max_accel),
      rclcpp::Parameter("takeoff_position_tolerance", position_tolerance),
      rclcpp::Parameter("takeoff_velocity_tolerance", velocity_tolerance),
      rclcpp::Parameter("takeoff_settle_time", settle_time),
      rclcpp::Parameter("takeoff_timeout", 15.0),
      rclcpp::Parameter("takeoff_max_overshoot", 1.0),
      rclcpp::Parameter("takeoff_low_saturation_timeout", 1.0),
      rclcpp::Parameter("takeoff_runaway_velocity", 1.0)
    });
  }

  void configure_takeoff_safety(
    double timeout,
    double max_overshoot,
    double low_saturation_timeout,
    double runaway_velocity)
  {
    set_parameters({
      rclcpp::Parameter("takeoff_timeout", timeout),
      rclcpp::Parameter("takeoff_max_overshoot", max_overshoot),
      rclcpp::Parameter(
        "takeoff_low_saturation_timeout", low_saturation_timeout),
      rclcpp::Parameter("takeoff_runaway_velocity", runaway_velocity)
    });
  }

  void set_low_throttle_saturation(bool saturated)
  {
    low_throttle_saturated_ = saturated;
  }

  bool takeoff_throttle_limit_active() const
  {
    return is_takeoff_throttle_limit_active();
  }

  roscopter_msgs::msg::ControllerCommand last_control_input;
  int reset_integrators_count = 0;
  int reset_vertical_integrators_count = 0;

private:
  roscopter_msgs::msg::ControllerCommand input_command_;
  rosflight_msgs::msg::Status status_;
  bool low_throttle_saturated_ = false;

  rosflight_msgs::msg::Command compute_offboard_control(
    roscopter_msgs::msg::ControllerCommand & input_cmd,
    double) override
  {
    last_control_input = input_cmd;
    rosflight_msgs::msg::Command output;
    output.u[2] = low_throttle_saturated_ ? 0.4 : 0.65;
    return output;
  }

  void reset_integrators() override
  {
    ++reset_integrators_count;
  }

  void reset_vertical_integrators() override
  {
    ++reset_vertical_integrators_count;
  }

  bool is_throttle_saturated_low() const override
  {
    return low_throttle_saturated_;
  }

  void update_gains() override {}
};

class ControllerTakeoffTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ = std::make_shared<TestController>();
    controller_->set_estimate(0.0, 0.0);
  }

  std::shared_ptr<TestController> controller_;
};

TEST_F(ControllerTakeoffTest, RampStartsNearZeroAndClimbsInNegativeNedDirection)
{
  controller_->configure_smooth_takeoff();
  controller_->start_takeoff();

  controller_->step(0.1);

  EXPECT_NEAR(controller_->last_control_input.cmd3, -0.01, 1e-6);
  EXPECT_LT(controller_->last_control_input.cmd3, 0.0);
}

TEST_F(ControllerTakeoffTest, RampNeverExceedsAccelerationLimit)
{
  const double dt = 0.1;
  const double max_accel = 0.1;
  controller_->configure_smooth_takeoff(max_accel);
  controller_->start_takeoff();

  double previous_velocity = 0.0;
  for (int i = 0; i < 100; ++i) {
    controller_->step(dt);
    const double velocity = controller_->last_control_input.cmd3;
    EXPECT_LE(std::abs(velocity - previous_velocity), max_accel * dt + 1e-6);
    previous_velocity = velocity;
  }
}

TEST_F(ControllerTakeoffTest, SwitchesFromVelocityRampToPositionCapture)
{
  controller_->configure_smooth_takeoff();
  controller_->start_takeoff();

  for (int i = 0; i < 50; ++i) {
    controller_->step(0.1);
  }
  EXPECT_EQ(
    controller_->last_control_input.mode,
    roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DVEL_YAW);
  EXPECT_EQ(
    controller_->get_takeoff_phase(),
    TestController::TAKEOFF_RAMP);

  controller_->set_estimate(-0.8, -0.2);
  controller_->step(0.1);

  EXPECT_EQ(
    controller_->get_takeoff_phase(),
    TestController::TAKEOFF_CAPTURE);
  EXPECT_EQ(
    controller_->last_control_input.mode,
    roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW);
  EXPECT_DOUBLE_EQ(controller_->last_control_input.cmd3, -2.0);
}

TEST_F(ControllerTakeoffTest, ZeroRemainingDistanceDoesNotProduceNan)
{
  controller_->configure_smooth_takeoff();
  controller_->start_takeoff();
  controller_->set_estimate(-2.0, 1.0);

  controller_->step(0.1);

  EXPECT_TRUE(std::isfinite(controller_->last_control_input.cmd3));
  EXPECT_EQ(
    controller_->last_control_input.mode,
    roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW);
  EXPECT_DOUBLE_EQ(controller_->last_control_input.cmd3, -2.0);
}

TEST_F(ControllerTakeoffTest, PositionAloneDoesNotCompleteSmoothTakeoff)
{
  controller_->configure_smooth_takeoff(0.1, 0.1, 0.1, 0.3);
  controller_->start_takeoff();
  controller_->set_estimate(-2.0, -0.5);

  for (int i = 0; i < 10; ++i) {
    controller_->step(0.1);
  }

  EXPECT_EQ(controller_->get_controller_state(), TestController::TAKEOFF);
  EXPECT_EQ(controller_->reset_vertical_integrators_count, 0);
}

TEST_F(ControllerTakeoffTest, ZeroSettleTimeStillRequiresPositionAndVelocity)
{
  controller_->configure_smooth_takeoff(0.1, 0.1, 0.1, 0.0);
  controller_->start_takeoff();
  controller_->set_estimate(0.0, 0.0);

  controller_->step(0.1);

  EXPECT_EQ(controller_->get_controller_state(), TestController::TAKEOFF);
  EXPECT_EQ(controller_->reset_vertical_integrators_count, 0);
}

TEST_F(ControllerTakeoffTest, CompletesOnlyAfterContinuousSettleTime)
{
  controller_->configure_smooth_takeoff(0.1, 0.1, 0.1, 0.3);
  controller_->start_takeoff();
  controller_->set_estimate(-2.0, 0.0);

  controller_->step(0.1);
  controller_->step(0.1);
  EXPECT_EQ(controller_->get_controller_state(), TestController::TAKEOFF);

  controller_->step(0.1);
  EXPECT_EQ(controller_->get_controller_state(), TestController::POSITION_HOLD);
  EXPECT_EQ(controller_->reset_vertical_integrators_count, 0);

  controller_->step(0.1);
  EXPECT_EQ(controller_->reset_vertical_integrators_count, 0);
}

TEST_F(
  ControllerTakeoffTest,
  TakeoffThrottleLimitPersistsThroughHoldAndClearsForOffboard)
{
  controller_->configure_smooth_takeoff(0.1, 0.1, 0.1, 0.1);
  controller_->start_takeoff();
  EXPECT_TRUE(controller_->takeoff_throttle_limit_active());

  controller_->set_estimate(-2.0, 0.0);
  controller_->step(0.1);
  ASSERT_EQ(
    controller_->get_controller_state(),
    TestController::POSITION_HOLD);
  EXPECT_TRUE(controller_->takeoff_throttle_limit_active());

  controller_->set_estimate(-2.0, 0.0, 3.1);
  controller_->step(0.1);
  ASSERT_EQ(controller_->get_controller_state(), TestController::OFFBOARD);
  EXPECT_FALSE(controller_->takeoff_throttle_limit_active());
}

TEST_F(ControllerTakeoffTest, SettleTimerResetsWhenConditionIsBroken)
{
  controller_->configure_smooth_takeoff(0.1, 0.1, 0.1, 0.3);
  controller_->start_takeoff();
  controller_->set_estimate(-2.0, 0.0);

  controller_->step(0.1);
  controller_->step(0.1);
  controller_->set_estimate(-2.0, -0.2);
  controller_->step(0.1);
  controller_->set_estimate(-2.0, 0.0);
  controller_->step(0.1);
  controller_->step(0.1);

  EXPECT_EQ(controller_->get_controller_state(), TestController::TAKEOFF);

  controller_->step(0.1);
  EXPECT_EQ(controller_->get_controller_state(), TestController::POSITION_HOLD);
}

TEST_F(ControllerTakeoffTest, DisabledSmoothTakeoffPreservesLegacyBehavior)
{
  controller_->set_parameters({
    rclcpp::Parameter("smooth_takeoff_enabled", false),
    rclcpp::Parameter("takeoff_d_pos", -2.0),
    rclcpp::Parameter("takeoff_d_vel", -0.5),
    rclcpp::Parameter("takeoff_height_threshold", 1.0)
  });
  controller_->start_takeoff();

  controller_->step(0.1);
  EXPECT_DOUBLE_EQ(controller_->last_control_input.cmd3, -0.5);

  controller_->set_estimate(-2.0, -5.0);
  controller_->step(0.1);
  EXPECT_EQ(controller_->get_controller_state(), TestController::POSITION_HOLD);
  EXPECT_EQ(controller_->reset_vertical_integrators_count, 1);
}

TEST_F(ControllerTakeoffTest, PersistentLowThrottleSaturationAbortsTakeoff)
{
  controller_->configure_smooth_takeoff();
  controller_->configure_takeoff_safety(10.0, 10.0, 0.3, 5.0);
  controller_->start_takeoff();
  controller_->set_low_throttle_saturation(true);

  controller_->set_estimate(-0.2, -0.1);
  controller_->step(0.1);
  controller_->set_estimate(-0.3, -0.2);
  controller_->step(0.1);
  controller_->set_estimate(-0.4, -0.3);
  controller_->step(0.1);

  EXPECT_EQ(
    controller_->get_controller_state(),
    TestController::TAKEOFF_ABORT);
  EXPECT_EQ(
    controller_->get_takeoff_abort_reason(),
    TestController::TAKEOFF_ABORT_LOW_THROTTLE_SATURATION);
  EXPECT_EQ(controller_->reset_vertical_integrators_count, 1);
}

TEST_F(ControllerTakeoffTest, RunawayUpwardVelocityAbortsTakeoff)
{
  controller_->configure_smooth_takeoff();
  controller_->configure_takeoff_safety(10.0, 10.0, 10.0, 1.0);
  controller_->start_takeoff();
  controller_->set_estimate(-0.5, -1.1);

  controller_->step(0.1);

  EXPECT_EQ(
    controller_->get_controller_state(),
    TestController::TAKEOFF_ABORT);
  EXPECT_EQ(
    controller_->get_takeoff_abort_reason(),
    TestController::TAKEOFF_ABORT_RUNAWAY_VELOCITY);
}

TEST_F(ControllerTakeoffTest, ExcessiveAltitudeOvershootAbortsTakeoff)
{
  controller_->configure_smooth_takeoff();
  controller_->configure_takeoff_safety(10.0, 0.5, 10.0, 10.0);
  controller_->start_takeoff();
  controller_->set_estimate(-2.6, -0.2);

  controller_->step(0.1);

  EXPECT_EQ(
    controller_->get_controller_state(),
    TestController::TAKEOFF_ABORT);
  EXPECT_EQ(
    controller_->get_takeoff_abort_reason(),
    TestController::TAKEOFF_ABORT_ALTITUDE_OVERSHOOT);
}

TEST_F(ControllerTakeoffTest, TakeoffTimeoutUsesFallbackTransition)
{
  controller_->configure_smooth_takeoff();
  controller_->configure_takeoff_safety(0.3, 10.0, 10.0, 10.0);
  controller_->start_takeoff();

  controller_->step(0.1);
  controller_->step(0.1);
  controller_->step(0.1);

  EXPECT_EQ(
    controller_->get_controller_state(),
    TestController::TAKEOFF_ABORT);
  EXPECT_EQ(
    controller_->get_takeoff_phase(),
    TestController::TAKEOFF_PHASE_ABORTED);
  EXPECT_EQ(
    controller_->get_takeoff_abort_reason(),
    TestController::TAKEOFF_ABORT_TIMEOUT);
  EXPECT_EQ(controller_->reset_vertical_integrators_count, 1);
}

TEST_F(ControllerTakeoffTest, SmoothTakeoffCannotRemainActiveIndefinitely)
{
  controller_->configure_smooth_takeoff();
  controller_->configure_takeoff_safety(0.5, 10.0, 10.0, 10.0);
  controller_->start_takeoff();

  for (int i = 0; i < 20; ++i) {
    controller_->step(0.1);
  }

  EXPECT_NE(controller_->get_controller_state(), TestController::TAKEOFF);
}

TEST_F(ControllerTakeoffTest, LegacyModeDoesNotEnableNewSafetyTransitions)
{
  controller_->set_parameters({
    rclcpp::Parameter("smooth_takeoff_enabled", false),
    rclcpp::Parameter("takeoff_d_pos", -2.0),
    rclcpp::Parameter("takeoff_d_vel", -0.5),
    rclcpp::Parameter("takeoff_height_threshold", 0.1),
    rclcpp::Parameter("takeoff_timeout", 0.1),
    rclcpp::Parameter("takeoff_max_overshoot", 0.1),
    rclcpp::Parameter("takeoff_low_saturation_timeout", 0.1),
    rclcpp::Parameter("takeoff_runaway_velocity", 0.1)
  });
  controller_->start_takeoff();
  controller_->set_low_throttle_saturation(true);
  controller_->set_estimate(-4.0, -5.0);

  for (int i = 0; i < 10; ++i) {
    controller_->step(0.1);
  }

  EXPECT_EQ(controller_->get_controller_state(), TestController::TAKEOFF);
  EXPECT_EQ(
    controller_->get_takeoff_abort_reason(),
    TestController::TAKEOFF_ABORT_NONE);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
