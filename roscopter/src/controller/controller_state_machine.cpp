#include <controller/controller_state_machine.hpp>

#include <algorithm>
#include <cmath>

using std::placeholders::_1;

namespace roscopter
{

ControllerStateMachine::ControllerStateMachine()
  : ControllerROS(),
    state_transition_(false),
    state_(DISARM),
    do_land_(false),
    takeoff_velocity_setpoint_(0.0),
    takeoff_settle_elapsed_(0.0)
{
  declare_params();
  params.set_parameters();
}

void ControllerStateMachine::declare_params()
{
  params.declare_double("takeoff_d_pos", -10.0);
  params.declare_double("takeoff_height_threshold", 1.0);
  params.declare_double("takeoff_d_vel", -0.5);
  params.declare_double("takeoff_landing_pos_hold_time", 3.0);
  params.declare_bool("smooth_takeoff_enabled", false);
  params.declare_double("takeoff_max_accel", 0.10);
  params.declare_double("takeoff_position_tolerance", 0.10);
  params.declare_double("takeoff_velocity_tolerance", 0.10);
  params.declare_double("takeoff_settle_time", 1.0);
}

void ControllerStateMachine::update_gains() {
  // No gains in the state machine need to be updated when parameters are changed. Do nothing.
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_state(roscopter_msgs::msg::ControllerCommand & input_cmd, rosflight_msgs::msg::Status & status_msg, double dt)
{
  rosflight_msgs::msg::Command output_command;

  // Make sure dt is over a threshold so the PID loops don't blow up
  if(dt <= 0.0000001) {
    RCLCPP_WARN_STREAM(this->get_logger(), "dt <= 0.0000001");
    return output_command;
  }

  // Calculate the output command based on the state machine
  // TODO: disarm state is detriggered if input_cmd has valid command. This currently won't ever get reset to false after receiving a valid one. Unless high level controller sends it.
  // TODO: Implement a timer, etc. to reset it after a certain time?
  switch (state_) {
    case DISARM:
      manage_disarm(status_msg.armed, input_cmd.cmd_valid);
      break;

    case TAKEOFF:
      RCLCPP_INFO_STREAM_EXPRESSION(this->get_logger(), state_transition_, "TAKEOFF mode");
      state_transition_ = false;

      output_command = manage_takeoff(dt);
      break;

    case OFFBOARD:
      RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), state_transition_, "OFFBOARD CONTROLLER ACTIVE");
      state_transition_ = false;

      output_command = compute_offboard_control(input_cmd, dt);

      if (!input_cmd.cmd_valid) {
        state_ = POSITION_HOLD;
      }
      break;

    case POSITION_HOLD:
      RCLCPP_INFO_STREAM_EXPRESSION(this->get_logger(), state_transition_, "POSITION_HOLD mode");
      state_transition_ = false;

      output_command = manage_position_hold(dt);
      break;

      // TODO: Currently nothing is available to have the vehicle land. It never will reach this state.
    case LANDING:
      RCLCPP_INFO_STREAM_EXPRESSION(this->get_logger(), state_transition_, "LANDING mode");
      state_transition_ = false;

      output_command = manage_landing();
      break;

    default:
      state_ = DISARM;
      break;
  }

  // Check to see if the disarmed
  if (!status_msg.armed) {
    state_ = DISARM;
  }

  // Check to see if control is valid
  // TODO: This will cause the state machine to reset after receiving invalid input commands... Figure out how to better handle this.
  // if (!input_cmd.cmd_valid && state_ != POSITION_HOLD) {
  //   state_ = POSITION_HOLD;
  //   RCLCPP_WARN_STREAM(this->get_logger(), "Entering POSITION_HOLD due to invalid input commands.");
  // }

  return output_command;
}

uint8_t ControllerStateMachine::get_controller_state() const
{
  return static_cast<uint8_t>(state_);
}

double ControllerStateMachine::get_takeoff_velocity_setpoint() const
{
  return takeoff_velocity_setpoint_;
}

void ControllerStateMachine::manage_disarm(bool armed, bool cmd_valid)
{
  if (armed && cmd_valid) {
    // Change the state appropriately and set the takeoff positions
    state_ = TAKEOFF;
    takeoff_n_pos_ = xhat_.p_n;
    takeoff_e_pos_ = xhat_.p_e;
    takeoff_yaw_ = xhat_.psi;
    takeoff_velocity_setpoint_ = 0.0;
    takeoff_settle_elapsed_ = 0.0;
  }
  else {
    RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), !state_transition_, 
      "OFFBOARD CONTROLLER INACTIVE");
    state_transition_ = true;

    reset_integrators();
  }
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_takeoff(double dt)
{
  double takeoff_d_pos = params.get_double("takeoff_d_pos");
  double takeoff_height_threshold = params.get_double("takeoff_height_threshold");
  double takeoff_d_vel = params.get_double("takeoff_d_vel");
  bool smooth_takeoff_enabled = params.get_bool("smooth_takeoff_enabled");

  rosflight_msgs::msg::Command output_cmd;
  bool takeoff_complete = false;

  if (smooth_takeoff_enabled) {
    takeoff_velocity_setpoint_ = update_smooth_takeoff_velocity(
      takeoff_d_pos,
      takeoff_d_vel,
      params.get_double("takeoff_max_accel"),
      dt);
  } else {
    // Preserve the original fixed-velocity takeoff behavior.
    takeoff_velocity_setpoint_ = takeoff_d_vel;
  }

  // Create high_level_command
  roscopter_msgs::msg::ControllerCommand input_cmd;
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DVEL_YAW;
  input_cmd.cmd1 = takeoff_n_pos_;
  input_cmd.cmd2 = takeoff_e_pos_;
  input_cmd.cmd3 = takeoff_velocity_setpoint_;
  input_cmd.cmd4 = takeoff_yaw_;

  // Compute command
  output_cmd = compute_offboard_control(input_cmd, dt);

  if (smooth_takeoff_enabled) {
    const double position_error_d = takeoff_d_pos - xhat_.p_d;
    const double velocity_d = calculate_inertial_down_velocity();
    const bool position_settled =
      std::abs(position_error_d) <= params.get_double("takeoff_position_tolerance");
    const bool velocity_settled =
      std::abs(velocity_d) <= params.get_double("takeoff_velocity_tolerance");

    if (position_settled && velocity_settled) {
      takeoff_settle_elapsed_ += dt;
    } else {
      takeoff_settle_elapsed_ = 0.0;
    }

    takeoff_complete =
      takeoff_settle_elapsed_ >= std::max(0.0, params.get_double("takeoff_settle_time"));
  } else {
    // Preserve the original position-only completion check.
    takeoff_complete = std::abs(xhat_.p_d - takeoff_d_pos) <= takeoff_height_threshold;
  }

  // Change state
  if (takeoff_complete) {
    transition_to_position_hold();
  }
  return output_cmd;
}

double ControllerStateMachine::calculate_inertial_down_velocity() const
{
  const double qw = xhat_.quat.w;
  const double qx = xhat_.quat.x;
  const double qy = xhat_.quat.y;
  const double qz = xhat_.quat.z;

  // Third row of the body-to-NED rotation matrix.
  return
    2.0 * (qx * qz - qw * qy) * xhat_.v_x +
    2.0 * (qy * qz + qw * qx) * xhat_.v_y +
    (1.0 - 2.0 * (qx * qx + qy * qy)) * xhat_.v_z;
}

double ControllerStateMachine::update_smooth_takeoff_velocity(
  double target_position_d,
  double cruise_velocity_d,
  double max_accel,
  double dt)
{
  const double position_error_d = target_position_d - xhat_.p_d;
  const double remaining_distance = std::abs(position_error_d);
  const double accel_limit = std::max(0.0, max_accel);
  const double safe_dt = std::max(0.0, dt);

  // sqrt() is guarded so a zero remaining distance and invalid negative
  // acceleration cannot produce NaN.
  const double brake_speed = std::sqrt(std::max(0.0, 2.0 * accel_limit * remaining_distance));
  const double target_speed = std::min(std::abs(cruise_velocity_d), brake_speed);

  double desired_velocity_d = 0.0;
  if (position_error_d < 0.0) {
    desired_velocity_d = -target_speed;  // Up is negative in NED.
  } else if (position_error_d > 0.0) {
    desired_velocity_d = target_speed;
  }

  const double max_velocity_delta = accel_limit * safe_dt;
  const double velocity_delta = desired_velocity_d - takeoff_velocity_setpoint_;
  const double limited_delta = std::max(
    -max_velocity_delta,
    std::min(max_velocity_delta, velocity_delta));

  takeoff_velocity_setpoint_ += limited_delta;
  return takeoff_velocity_setpoint_;
}

void ControllerStateMachine::transition_to_position_hold()
{
  state_ = POSITION_HOLD;
  start_position_hold_time_ =
    xhat_.header.stamp.sec + xhat_.header.stamp.nanosec * 1e-9;
  state_transition_ = true;
  reset_vertical_integrators();
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_position_hold(double dt)
{
  double takeoff_d_pos = params.get_double("takeoff_d_pos");
  double takeoff_landing_pos_hold_time = params.get_double("takeoff_landing_pos_hold_time");

  rosflight_msgs::msg::Command output_cmd;
  bool start_landing = false;
  bool start_offboard = false;

  // Create high level command
  roscopter_msgs::msg::ControllerCommand input_cmd;
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW;
  input_cmd.cmd1 = takeoff_n_pos_;
  input_cmd.cmd2 = takeoff_e_pos_;
  input_cmd.cmd3 = takeoff_d_pos;
  input_cmd.cmd4 = takeoff_yaw_;

  // Compute command
  output_cmd = compute_offboard_control(input_cmd, dt);

  // Check to see if the hold time is up
  double now = xhat_.header.stamp.sec + xhat_.header.stamp.nanosec * 1e-9;
  double elapsed_time = now - start_position_hold_time_;
  if (elapsed_time >= takeoff_landing_pos_hold_time) {
    if (do_land_) {
      start_landing = true;
    }
    else { start_offboard = true; }
  }

  // Transition states, as appropriate
  if (start_landing) {
    state_ = LANDING;
    state_transition_ = true;
  }
  else if (start_offboard) {
    state_ = OFFBOARD;
    state_transition_ = true;
  }

  return output_cmd;
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_landing()
{
  rosflight_msgs::msg::Command output_cmd;
  bool landing_complete = false;

  if (landing_complete) {
    state_ = DISARM;
    state_transition_ = true;
  }
  return output_cmd;
}

}  // namespace controller
