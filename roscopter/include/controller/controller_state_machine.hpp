#ifndef CONTROLLER_STATE_MACHINE_HPP
#define CONTROLLER_STATE_MACHINE_HPP

#include <controller/controller_ros.hpp>
#include <controller/simple_pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/controller_command.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>

#include <cstdint>

using std::placeholders::_1;

namespace roscopter
{

class ControllerStateMachine : public ControllerROS
{

public:

  enum ControllerState : uint8_t
  {
    DISARM,
    TAKEOFF,
    OFFBOARD,
    POSITION_HOLD,
    LANDING,
    TAKEOFF_ABORT
  };

  enum TakeoffPhase : uint8_t
  {
    TAKEOFF_PHASE_INACTIVE,
    TAKEOFF_RAMP,
    TAKEOFF_CAPTURE,
    TAKEOFF_PHASE_ABORTED
  };

  enum TakeoffAbortReason : uint8_t
  {
    TAKEOFF_ABORT_NONE,
    TAKEOFF_ABORT_TIMEOUT,
    TAKEOFF_ABORT_LOW_THROTTLE_SATURATION,
    TAKEOFF_ABORT_RUNAWAY_VELOCITY,
    TAKEOFF_ABORT_ALTITUDE_OVERSHOOT
  };

  ControllerStateMachine();

  uint8_t get_controller_state() const;
  double get_takeoff_velocity_setpoint() const;
  uint8_t get_takeoff_phase() const;
  double get_takeoff_elapsed_time() const;
  double get_low_throttle_saturation_elapsed() const;
  uint8_t get_takeoff_abort_reason() const;

protected:
  rosflight_msgs::msg::Command manage_state(
    roscopter_msgs::msg::ControllerCommand & input_cmd,
    rosflight_msgs::msg::Status & status_msg,
    double dt) override;
  bool is_takeoff_throttle_limit_active() const;

private:
  // Parameters
  double min_altitude_;
  bool state_transition_;
  ControllerState state_;
  double takeoff_n_pos_;
  double takeoff_e_pos_;
  double takeoff_yaw_;
  double start_position_hold_time_;
  bool do_land_;
  double takeoff_start_d_pos_;
  double takeoff_velocity_setpoint_;
  double takeoff_settle_elapsed_;
  double takeoff_elapsed_time_;
  double low_throttle_saturation_elapsed_;
  double low_saturation_start_upward_speed_;
  TakeoffPhase takeoff_phase_;
  TakeoffAbortReason takeoff_abort_reason_;
  bool takeoff_throttle_limit_active_;

  // Functions
  void declare_params();

  void manage_disarm(bool armed, bool cmd_valid);
  rosflight_msgs::msg::Command manage_takeoff(double dt);
  rosflight_msgs::msg::Command manage_takeoff_abort(double dt);
  rosflight_msgs::msg::Command manage_position_hold(double dt);
  rosflight_msgs::msg::Command manage_landing();
  double calculate_inertial_down_velocity() const;
  double update_smooth_takeoff_velocity(
    double target_position_d,
    double cruise_velocity_d,
    double max_accel,
    double dt);
  bool should_enter_takeoff_capture(
    double target_position_d,
    double cruise_velocity_d,
    double max_accel);
  void update_takeoff_safety(double velocity_d, double dt);
  void abort_takeoff(TakeoffAbortReason reason);
  void transition_to_position_hold();

  virtual void update_gains();
  virtual rosflight_msgs::msg::Command compute_offboard_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt) = 0;
  virtual void reset_integrators() = 0;
  virtual void reset_vertical_integrators() = 0;
  virtual bool is_throttle_saturated_low() const;
  virtual void finalize_control_cycle(const rosflight_msgs::msg::Command & output_cmd);
};

}   // namespace controller

#endif
