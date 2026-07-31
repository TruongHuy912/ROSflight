#!/usr/bin/env python3

import json
import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters

from roscopter_msgs.msg import ControllerCommand
from roscopter_msgs.msg import State
from roscopter_msgs.msg import VerticalControlDebug
from rosflight_msgs.msg import Status
from rosflight_msgs.srv import ParamFile
from rosflight_msgs.srv import ParamGet
from std_srvs.srv import Trigger


TARGET_ALTITUDE = 2.0
MAX_ALTITUDE = TARGET_ALTITUDE + 1.0
MAX_TAKEOFF_DURATION = 15.0
MAX_LOW_SATURATION_DURATION = 1.0
MAX_NEAR_TARGET_VERTICAL_SPEED = 1.0


def message_time(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def generate_test_description():
    rosflight_sim_dir = get_package_share_directory("rosflight_sim")
    roscopter_dir = get_package_share_directory("roscopter")
    dynamics_params = os.path.join(
        rosflight_sim_dir, "params", "multirotor_dynamics.yaml")

    common_sim_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rosflight_sim_dir,
                "launch",
                "common_nodes_standalone.launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "dynamics_param_file": dynamics_params,
        }.items())

    forces_and_moments = Node(
        package="rosflight_sim",
        executable="multirotor_forces_and_moments",
        name="multirotor_forces_and_moments",
        output="screen",
        parameters=[dynamics_params])

    dynamics = Node(
        package="rosflight_sim",
        executable="standalone_dynamics",
        name="standalone_dynamics",
        output="screen",
        parameters=[dynamics_params])

    truth_transcriber = Node(
        package="roscopter_sim",
        executable="sim_state_transcriber",
        name="roscopter_truth",
        output="screen")

    controller = Node(
        package="roscopter",
        executable="controller",
        name="controller",
        output="screen",
        parameters=[
            os.path.join(roscopter_dir, "params", "multirotor.yaml"),
        ],
        remappings=[("estimated_state", "/sim/roscopter/state")])

    return (
        launch.LaunchDescription([
            common_sim_nodes,
            forces_and_moments,
            dynamics,
            truth_transcriber,
            controller,
            launch_testing.actions.ReadyToTest(),
        ]),
        {})


class SmoothTakeoffMonitor(RclpyNode):
    def __init__(self):
        super().__init__("smooth_takeoff_integration_monitor")
        self.truth_samples = []
        self.debug_samples = []
        self.status = None
        self.create_subscription(
            State, "/sim/roscopter/state", self._truth_callback, 100)
        self.create_subscription(
            VerticalControlDebug,
            "/controller/vertical_debug",
            self._debug_callback,
            100)
        self.create_subscription(Status, "/status", self._status_callback, 10)
        self.command_pub = self.create_publisher(
            ControllerCommand, "/high_level_command", 10)

    def _truth_callback(self, msg):
        self.truth_samples.append((time.monotonic(), msg))

    def _debug_callback(self, msg):
        self.debug_samples.append((time.monotonic(), msg))

    def _status_callback(self, msg):
        self.status = msg

    def publish_valid_command(self):
        command = ControllerCommand()
        command.mode = ControllerCommand.MODE_NPOS_EPOS_DPOS_YAW
        command.cmd_valid = True
        command.cmd3 = -TARGET_ALTITUDE
        self.command_pub.publish(command)


class TestSmoothTakeoffClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = SmoothTakeoffMonitor()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout, publish_command=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish_command:
                self.node.publish_valid_command()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _call(self, service_name, service_type, request, timeout=15.0):
        client = self.node.create_client(service_type, service_name)
        try:
            self.assertTrue(
                client.wait_for_service(timeout_sec=timeout),
                f"service unavailable: {service_name}")
            future = client.call_async(request)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and not future.done():
                rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertTrue(future.done(), f"service timeout: {service_name}")
            return future.result()
        finally:
            self.node.destroy_client(client)

    def _wait_for_all_firmware_params(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            response = self._call(
                "/all_params_received",
                Trigger,
                Trigger.Request(),
                timeout=2.0)
            if response.success:
                return True
            self._spin_until(lambda: False, 0.25)
        return False

    def _wait_for_firmware_param(self, name, expected, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            request = ParamGet.Request()
            request.name = name
            response = self._call(
                "/param_get", ParamGet, request, timeout=2.0)
            if response.exists and abs(response.value - expected) < 1e-6:
                return True
            self._spin_until(lambda: False, 0.25)
        return False

    def _configure_controller(self):
        values = {
            "smooth_takeoff_enabled": True,
            # Physical hover feed-forward for the standalone ROSflight
            # multirotor. This is a simulation model parameter, not a PID gain.
            "equilibrium_throttle": 0.34,
            "takeoff_d_pos": -TARGET_ALTITUDE,
            "takeoff_d_vel": -0.2,
            "takeoff_max_accel": 0.1,
            "takeoff_position_tolerance": 0.1,
            "takeoff_velocity_tolerance": 0.1,
            "takeoff_settle_time": 1.0,
            "takeoff_timeout": MAX_TAKEOFF_DURATION,
            "takeoff_max_overshoot": 1.0,
            "takeoff_low_saturation_timeout":
                MAX_LOW_SATURATION_DURATION,
            "takeoff_runaway_velocity": 1.0,
            # Explicit simulation-only override. The normal motor floor
            # remains unchanged and is inherited when this is omitted.
            "takeoff_min_throttle": 0.15,
            "takeoff_landing_pos_hold_time": 60.0,
        }
        set_request = SetParameters.Request()
        set_request.parameters = [
            Parameter(name, value=value).to_parameter_msg()
            for name, value in values.items()
        ]
        set_response = self._call(
            "/controller/set_parameters",
            SetParameters,
            set_request,
            timeout=10.0)
        self.assertTrue(
            all(result.successful for result in set_response.results),
            "controller rejected simulation takeoff parameters")

        get_request = GetParameters.Request()
        get_request.names = [
            "smooth_takeoff_enabled",
            "equilibrium_throttle",
            "min_throttle",
            "takeoff_min_throttle"]
        get_response = self._call(
            "/controller/get_parameters",
            GetParameters,
            get_request,
            timeout=10.0)
        self.assertTrue(get_response.values[0].bool_value)
        self.assertAlmostEqual(get_response.values[1].double_value, 0.34)
        self.assertAlmostEqual(get_response.values[2].double_value, 0.4)
        self.assertAlmostEqual(get_response.values[3].double_value, 0.15)

    def test_closed_loop_smooth_takeoff(self):
        firmware_params = os.path.join(
            get_package_share_directory("rosflight_sim"),
            "params",
            "multirotor_firmware",
            "multirotor_combined.yaml")

        self.assertTrue(
            self._spin_until(
                lambda: len(self.node.truth_samples) > 20, 20.0),
            "ROSflight simulator did not publish ground truth")
        self._configure_controller()
        self.assertTrue(
            self._wait_for_all_firmware_params(),
            "rosflight_io did not receive the complete firmware parameter set")

        load_request = ParamFile.Request()
        load_request.filename = firmware_params
        load_response = self._call(
            "/param_load_from_file", ParamFile, load_request)
        self.assertTrue(load_response.success, "firmware parameter load failed")
        self.assertTrue(
            self._wait_for_firmware_param("RC_ARM_CHN", 4.0),
            "firmware arm-channel parameter was not applied")
        self.assertTrue(
            self._wait_for_firmware_param("PRIMARY_MIXER", 11.0),
            "firmware mixer parameter was not applied")

        self._call("/calibrate_imu", Trigger, Trigger.Request())
        self._spin_until(lambda: False, 3.0)
        self.assertTrue(
            self._spin_until(lambda: self.node.status is not None, 5.0),
            "firmware status was not published")

        arm_response = self._call(
            "/toggle_arm", Trigger, Trigger.Request())
        self.assertTrue(arm_response.success, "simulation arm failed")
        self.assertTrue(
            self._spin_until(
                lambda: self.node.status is not None and
                self.node.status.armed,
                5.0,
                publish_command=True),
            "firmware never reported armed")

        override_response = self._call(
            "/toggle_override", Trigger, Trigger.Request())
        self.assertTrue(
            override_response.success, "offboard override failed")

        self.assertTrue(
            self._spin_until(
                lambda: any(
                    sample.controller_state == 1
                    for _, sample in self.node.debug_samples),
                5.0,
                publish_command=True),
            "controller never entered TAKEOFF")

        takeoff_start = next(
            stamp for stamp, sample in self.node.debug_samples
            if sample.controller_state == 1)
        finished = self._spin_until(
            lambda: self.node.debug_samples and
            self.node.debug_samples[-1][1].controller_state != 1,
            MAX_TAKEOFF_DURATION + 2.0,
            publish_command=True)
        takeoff_end = time.monotonic()

        takeoff_end_sim = next(
            message_time(sample) for _, sample in self.node.debug_samples
            if sample.controller_state != 1)
        # Continue for two simulation seconds to confirm capture remains stable.
        self._spin_until(
            lambda: self.node.debug_samples and
            message_time(self.node.debug_samples[-1][1]) >=
            takeoff_end_sim + 2.0,
            3.0,
            publish_command=True)

        try:
            self._call("/toggle_arm", Trigger, Trigger.Request(), timeout=5.0)
        except AssertionError:
            pass

        truth = [
            (stamp, msg) for stamp, msg in self.node.truth_samples
            if stamp >= takeoff_start]
        debug = [
            (stamp, msg) for stamp, msg in self.node.debug_samples
            if stamp >= takeoff_start]
        self.assertTrue(truth, "no ground-truth samples during takeoff")
        self.assertTrue(debug, "no vertical-debug samples during takeoff")

        altitudes = [-msg.p_d for _, msg in truth]
        peak_altitude = max(altitudes)
        overshoot = max(0.0, peak_altitude - TARGET_ALTITUDE)
        takeoff_start_sim = next(
            message_time(msg) for _, msg in debug
            if msg.controller_state == 1)
        takeoff_duration = takeoff_end_sim - takeoff_start_sim
        max_low_saturation = max(
            msg.low_throttle_saturation_elapsed for _, msg in debug)

        near_target_speeds = [
            abs(msg.v_d) for _, msg in debug
            if math.isfinite(msg.v_d) and
            abs((-msg.p_d) - TARGET_ALTITUDE) <= 0.5]
        max_near_target_speed = (
            max(near_target_speeds) if near_target_speeds else math.inf)
        all_vertical_speeds = [
            abs(msg.v_d) for _, msg in debug if math.isfinite(msg.v_d)]
        max_vertical_speed = max(all_vertical_speeds)
        low_saturation_percentage = 100.0 * sum(
            bool(msg.throttle_saturated_low) for _, msg in debug) / len(debug)
        high_saturation_percentage = 100.0 * sum(
            bool(msg.throttle_saturated_high) for _, msg in debug) / len(debug)

        state_durations = {}
        phase_durations = {}
        for (_, msg), (_, next_msg) in zip(debug, debug[1:]):
            sample_duration = max(
                0.0, message_time(next_msg) - message_time(msg))
            key = str(msg.controller_state)
            state_durations[key] = (
                state_durations.get(key, 0.0) + sample_duration)
            phase_key = str(msg.takeoff_phase)
            phase_durations[phase_key] = (
                phase_durations.get(phase_key, 0.0) + sample_duration)

        final_debug = debug[-1][1]
        last_takeoff_debug = [
            msg for _, msg in debug if msg.controller_state == 1][-1]
        first_post_takeoff_debug = next(
            msg for _, msg in debug if msg.controller_state != 1)
        metrics = {
            "target_altitude_m": TARGET_ALTITUDE,
            "peak_altitude_m": peak_altitude,
            "overshoot_m": overshoot,
            "maximum_vertical_velocity_mps": max_vertical_speed,
            "maximum_near_target_vertical_velocity_mps":
                max_near_target_speed,
            "low_throttle_saturation_percent":
                low_saturation_percentage,
            "high_throttle_saturation_percent":
                high_saturation_percentage,
            "maximum_continuous_low_saturation_s":
                max_low_saturation,
            "takeoff_duration_s": takeoff_duration,
            "settling_time_s": takeoff_duration,
            "time_in_controller_state_s": state_durations,
            "time_in_takeoff_phase_s": phase_durations,
            "final_controller_state": final_debug.controller_state,
            "final_takeoff_phase": final_debug.takeoff_phase,
            "takeoff_abort_reason": final_debug.takeoff_abort_reason,
            "last_takeoff_sample": {
                "p_d": last_takeoff_debug.p_d,
                "v_d": last_takeoff_debug.v_d,
                "throttle": last_takeoff_debug.actual_output_command_u2,
                "velocity_integrator":
                    last_takeoff_debug.velocity_integrator,
            },
            "first_post_takeoff_sample": {
                "p_d": first_post_takeoff_debug.p_d,
                "v_d": first_post_takeoff_debug.v_d,
                "throttle":
                    first_post_takeoff_debug.actual_output_command_u2,
                "velocity_integrator":
                    first_post_takeoff_debug.velocity_integrator,
            },
            "final_sample": {
                "p_d": final_debug.p_d,
                "v_d": final_debug.v_d,
                "throttle": final_debug.actual_output_command_u2,
                "velocity_integrator": final_debug.velocity_integrator,
            },
            "minimum_output_throttle": min(
                msg.actual_output_command_u2 for _, msg in debug),
            "maximum_output_throttle": max(
                msg.actual_output_command_u2 for _, msg in debug),
        }
        print("SMOOTH_TAKEOFF_METRICS=" + json.dumps(
            metrics, sort_keys=True))

        self.assertTrue(finished, "TAKEOFF exceeded 15 seconds")
        self.assertLessEqual(takeoff_duration, MAX_TAKEOFF_DURATION)
        self.assertLessEqual(peak_altitude, MAX_ALTITUDE)
        self.assertLessEqual(
            max_low_saturation, MAX_LOW_SATURATION_DURATION)
        self.assertLess(
            max_near_target_speed, MAX_NEAR_TARGET_VERTICAL_SPEED)
        self.assertEqual(
            final_debug.takeoff_abort_reason,
            VerticalControlDebug.TAKEOFF_ABORT_NONE)
        self.assertNotIn(final_debug.controller_state, (1, 5))
        self.assertGreaterEqual(
            metrics["minimum_output_throttle"], 0.15 - 1e-6)
        self.assertLess(
            first_post_takeoff_debug.actual_output_command_u2, 0.4)
        for _, sample in debug:
            if math.isfinite(sample.throttle_saturated):
                self.assertAlmostEqual(
                    sample.actual_output_command_u2,
                    sample.throttle_saturated,
                    places=6)
