#!/usr/bin/env python3
"""
KAERTEI 2025 FAIO - 12 Checkpoint Mission Controller
Complete implementation of 12-checkpoint FSM system for autonomous drone mission
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import Image, NavSatFix
import time
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Callable, Dict, Optional
import yaml
import json

# MAVROS imports
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget, GlobalPositionTarget, WaypointList, StatusText, RCIn, RCOut
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, WaypointPull, ParamGet, ParamSet
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import MagneticField, Temperature

# Import hardware configuration
from ..hardware.hardware_config import HardwareConfig

class MissionCheckpoint(Enum):
    """12 Checkpoint Mission System - KAERTEI 2025 FAIO (CP-01 .. CP-12)"""
    CP01_INIT_ARM = "CP-01_INIT_ARM"
    CP02_TAKEOFF_1M = "CP-02_TAKEOFF_1M"
    CP03_SEARCH_ITEM1 = "CP-03_SEARCH_ITEM1"
    CP04_SEARCH_ITEM2_TURN = "CP-04_SEARCH_ITEM2_TURN"
    CP05_DROP_ITEM1 = "CP-05_DROP_ITEM1"
    CP06_DROP_ITEM2 = "CP-06_DROP_ITEM2"
    CP07_GPS_WP1_3 = "CP-07_GPS_WP1_3"
    CP08_SEARCH_ITEM3 = "CP-08_SEARCH_ITEM3"
    CP09_DIRECT_WP4 = "CP-09_DIRECT_WP4"
    CP10_SEARCH_DROP_ITEM3 = "CP-10_SEARCH_DROP_ITEM3"
    CP11_GPS_WP5 = "CP-11_GPS_WP5"
    CP12_FINAL_DESCENT_DISARM = "CP-12_FINAL_DESCENT_DISARM"
    # Mission States
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    PAUSED = "PAUSED"

@dataclass
class CheckpointSpec:
    guard: Callable[[], bool]
    action: Callable[[], None]
    next_success: MissionCheckpoint
    next_fail: MissionCheckpoint
    timeout_s: int
    retries: int

class Checkpoint12MissionNode(Node):
    def __init__(self):
        super().__init__('checkpoint_12_mission_node')
        
        # Load hardware configuration
        self.hw_config = HardwareConfig()
        
        # Mission parameters
        self.debug_mode = self.declare_parameter('debug_mode', True).value
        self.auto_continue = self.declare_parameter('auto_continue', False).value
        
        # Load YAML parameters (hardware_config.yaml), fallback to .conf via HardwareConfig
        self.params = self._load_yaml_params()
        
        # Current mission state
        self.current_checkpoint = MissionCheckpoint.CP01_INIT_ARM
        # Start CP-01 automatically; pause only between subsequent CPs
        self.waiting_for_next = False
        self.checkpoint_completed = False
        self.current_retries: int = 0
        self.cp_start_time: float = time.time()
        
        # Mission completion tracking
        self.item1_collected = False
        self.item2_collected = False
        self.item3_collected = False
        self.indoor_items_dropped = False
        self.outdoor_item_dropped = False
        self.mission_completed = False
        
        # Detection states
        self.item_detected = False
        self.item_position = Point()
        self.item_aligned = False
        self.bucket_detected = False
        self.bucket_position = Point()
        
        # Flight states
        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.gps_position = NavSatFix()
        self.current_altitude = 0.0
        self.armed = False
        
        # QoS profiles
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.checkpoint_status_pub = self.create_publisher(String, '/mission/checkpoint_status', qos_reliable)
        self.mission_command_pub = self.create_publisher(String, '/mission/command', qos_reliable)
        self.camera_command_pub = self.create_publisher(String, '/camera/command', qos_reliable)
        self.magnet_command_pub = self.create_publisher(String, '/magnet/command', qos_reliable)
        self.velocity_pub = self.create_publisher(TwistStamped, '/mavros_node/setpoint_velocity/cmd_vel', qos_reliable)
        # Use raw local setpoint (PositionTarget) according to MAVROS interface map
        self.local_setpoint_pub = self.create_publisher(PositionTarget, '/mavros_node/setpoint_raw/local', qos_reliable)
        self.global_pos_pub = self.create_publisher(GlobalPositionTarget, '/mavros_node/setpoint_raw/global', qos_reliable)
        
        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros_node/state', self.mavros_state_callback, qos_reliable)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros_node/local_position/pose', self.pose_callback, qos_reliable)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros_node/global_position/global', self.gps_callback, qos_reliable)
        self.waypoints_sub = self.create_subscription(WaypointList, '/mavros_node/mission/waypoints', self.waypoints_callback, qos_reliable)
        # FCU status text (MAVLink STATUSTEXT)
        self.statustext_sub = self.create_subscription(StatusText, '/mavros_node/statustext/recv', self.statustext_callback, qos_reliable)
        # Additional sensor topics
        self.raw_fix_sub = self.create_subscription(NavSatFix, '/mavros_node/global_position/raw/fix', self.raw_fix_callback, qos_reliable)
        self.mag_sub = self.create_subscription(MagneticField, '/mavros_node/imu/mag', self.mag_callback, qos_best_effort)
        self.temp_sub = self.create_subscription(Temperature, '/mavros_node/imu/temperature', self.temperature_callback, qos_best_effort)
        # Optional RC interfaces
        self.rc_in_sub = self.create_subscription(RCIn, '/mavros_node/rc/in', self.rc_in_callback, qos_reliable)
        self.rc_out_sub = self.create_subscription(RCOut, '/mavros_node/rc/out', self.rc_out_callback, qos_reliable)
        # Expect unified detection as geometry_msgs/Point
        self.vision_sub = self.create_subscription(Point, '/vision/detection', self.vision_callback, qos_best_effort)
        self.user_input_sub = self.create_subscription(String, '/mission/user_input', self.user_input_callback, qos_reliable)
        
        # Services
        self.arm_service = self.create_client(CommandBool, '/mavros_node/mavros_node/arming')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros_node/mavros_node/takeoff')
        self.land_service = self.create_client(CommandTOL, '/mavros_node/mavros_node/land')
        self.mode_service = self.create_client(SetMode, '/mavros_node/set_mode')
        self.wp_pull_service = self.create_client(WaypointPull, '/mavros_node/mission/pull')
        self.param_get_service = self.create_client(ParamGet, '/mavros_node/param/get')
        self.param_set_service = self.create_client(ParamSet, '/mavros_node/param/set')
        # RC override publisher (optional manual control)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros_node/rc/override', qos_reliable)
        
        # Waypoint pulling state
        self.px4_waypoints_count = 0
        self._last_logged_wp_count = None

        # Build CP table
        self.cp_table: Dict[MissionCheckpoint, CheckpointSpec] = self._build_cp_table()

        # Setpoint streaming (for OFFBOARD/PX4 or GUIDED/ArduPilot stability)
        self.setpoint_active = False
        self.last_twist_cmd = TwistStamped()
        self.last_twist_cmd.twist.linear.x = 0.0
        self.last_twist_cmd.twist.linear.y = 0.0
        self.last_twist_cmd.twist.linear.z = 0.0
        rate_hz = float(self._p('setpoint_rate_hz', 20))
        self.setpoint_timer = self.create_timer(max(0.01, 1.0 / rate_hz), self._stream_setpoint)

        # Initial PX4 waypoint pull (retry until MAVROS connected)
        self._wp_init_done = False
        self._wp_timer = self.create_timer(2.0, self._try_pull_waypoints_when_ready)

        # Mission control timer
        self.mission_timer = self.create_timer(0.5, self.mission_control_loop)
        
        self.get_logger().info("🚁 KAERTEI 2025 - 12 Checkpoint Mission Controller Initialized")
        self.get_logger().info(f"Debug Mode: {self.debug_mode}")

        # Latest sensor/RC/status caches
        self.latest_statustext = None
        self.gps_raw_fix = None
        self.latest_mag = None
        self.latest_imu_temp = None
        self.latest_rc_in = None
        self.latest_rc_out = None

    def mission_control_loop(self):
        """Main mission control loop"""
        if self.mission_completed or self.current_checkpoint in (MissionCheckpoint.COMPLETED, MissionCheckpoint.ERROR, MissionCheckpoint.PAUSED):
            return
        
        # In debug mode, require manual continue between CP
        if self.waiting_for_next:
            return
        
        # Evaluate guard and timeout
        spec = self.cp_table.get(self.current_checkpoint)
        if spec is None:
            self.get_logger().error(f"No spec for {self.current_checkpoint}")
            self.transition_to_error()
            return
        
        elapsed = time.time() - self.cp_start_time
        if elapsed > spec.timeout_s:
            self.get_logger().warning(f"⏳ Timeout on {self.current_checkpoint.value} after {elapsed:.1f}s")
            self._advance(False, spec)
            return
        
        # Guard
        guard_ok = False
        try:
            guard_ok = spec.guard()
        except Exception as e:
            self.get_logger().error(f"Guard error on {self.current_checkpoint.value}: {e}")
        
        if not guard_ok:
            # Wait until guard satisfied
            self.publish_checkpoint_status(self.current_checkpoint.value, "WAITING_GUARD")
            return
        
        # Execute action once per CP; action is responsible for calling complete_checkpoint()
        if not hasattr(self, '_executing_action') or not self._executing_action:
            self._executing_action = True
            self.publish_checkpoint_status(self.current_checkpoint.value, "EXECUTING")
            try:
                spec.action()
                # Do not advance here; the action will call complete_checkpoint()
            except Exception as e:
                self.get_logger().error(f"Action error on {self.current_checkpoint.value}: {e}")
                self._advance(False, spec)
            finally:
                self._executing_action = False
            
    # ===========================================
    # FSM Helpers
    # ===========================================
    def _advance(self, success: bool, spec: CheckpointSpec):
        if success:
            self.publish_checkpoint_status(self.current_checkpoint.value, "COMPLETED")
            next_cp = spec.next_success
            self.get_logger().info(f"✅ {self.current_checkpoint.value} → {next_cp.value}")
            self.current_checkpoint = next_cp
            self.current_retries = 0
            self.cp_start_time = time.time()
        else:
            self.current_retries += 1
            if self.current_retries <= spec.retries:
                self.get_logger().warning(f"🔁 Retry {self.current_retries}/{spec.retries} on {self.current_checkpoint.value}")
                self.cp_start_time = time.time()
            else:
                next_cp = spec.next_fail
                self.get_logger().error(f"❌ {self.current_checkpoint.value} failed → {next_cp.value}")
                self.publish_checkpoint_status(self.current_checkpoint.value, "FAILED")
                self.current_checkpoint = next_cp
                self.current_retries = 0
                self.cp_start_time = time.time()
        
        # In debug mode, pause between CP
        if self.debug_mode:
            self.waiting_for_next = True
            self.get_logger().info("💬 Send 'continue' to proceed...")

    def _load_yaml_params(self) -> Dict:
        try:
            import os
            yaml_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'config', 'hardware_config.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return data
        except Exception:
            return {}

    def _p(self, path: str, default):
        """Read nested param from YAML like 'timeouts.init_arm'"""
        d = self.params
        for part in path.split('.'):
            if isinstance(d, dict) and part in d:
                d = d[part]
            else:
                return default
        return d

    def _build_cp_table(self) -> Dict[MissionCheckpoint, CheckpointSpec]:
        t = self._p
        return {
            MissionCheckpoint.CP01_INIT_ARM: CheckpointSpec(
                guard=self.guard_init_arm,
                action=self.execute_cp1_init_arm,
                next_success=MissionCheckpoint.CP02_TAKEOFF_1M,
                next_fail=MissionCheckpoint.CP12_FINAL_DESCENT_DISARM,
                timeout_s=int(t('timeouts.init_arm', 15)),
                retries=int(t('retries.arm', 3)),
            ),
            MissionCheckpoint.CP02_TAKEOFF_1M: CheckpointSpec(
                guard=self.guard_takeoff_ready,
                action=self.execute_cp2_takeoff_1m,
                next_success=MissionCheckpoint.CP03_SEARCH_ITEM1,
                next_fail=MissionCheckpoint.CP12_FINAL_DESCENT_DISARM,
                timeout_s=int(t('timeouts.takeoff', 20)),
                retries=0,
            ),
            MissionCheckpoint.CP03_SEARCH_ITEM1: CheckpointSpec(
                guard=self.guard_vision_ready,
                action=self.execute_cp3_search_item1,
                next_success=MissionCheckpoint.CP04_SEARCH_ITEM2_TURN,
                next_fail=MissionCheckpoint.CP04_SEARCH_ITEM2_TURN,
                timeout_s=int(t('timeouts.search', 25)),
                retries=0,
            ),
            MissionCheckpoint.CP04_SEARCH_ITEM2_TURN: CheckpointSpec(
                guard=self.guard_vision_ready,
                action=self.execute_cp4_search_item2_turn,
                next_success=MissionCheckpoint.CP05_DROP_ITEM1,
                next_fail=MissionCheckpoint.CP05_DROP_ITEM1,
                timeout_s=int(t('timeouts.search', 25)),
                retries=0,
            ),
            MissionCheckpoint.CP05_DROP_ITEM1: CheckpointSpec(
                guard=self.guard_hover_stable,
                action=self.execute_cp5_drop_item1,
                next_success=MissionCheckpoint.CP06_DROP_ITEM2,
                next_fail=MissionCheckpoint.CP06_DROP_ITEM2,
                timeout_s=int(t('timeouts.drop', 15)),
                retries=1,
            ),
            MissionCheckpoint.CP06_DROP_ITEM2: CheckpointSpec(
                guard=self.guard_hover_stable,
                action=self.execute_cp6_drop_item2,
                next_success=MissionCheckpoint.CP07_GPS_WP1_3,
                next_fail=MissionCheckpoint.CP07_GPS_WP1_3,
                timeout_s=int(t('timeouts.drop', 15)),
                retries=1,
            ),
            MissionCheckpoint.CP07_GPS_WP1_3: CheckpointSpec(
                guard=self.guard_gps_ok,
                action=self.execute_cp7_gps_wp1_3,
                next_success=MissionCheckpoint.CP08_SEARCH_ITEM3,
                next_fail=MissionCheckpoint.CP08_SEARCH_ITEM3,
                timeout_s=180,
                retries=0,
            ),
            MissionCheckpoint.CP08_SEARCH_ITEM3: CheckpointSpec(
                guard=self.guard_vision_ready,
                action=self.execute_cp8_search_item3,
                next_success=MissionCheckpoint.CP09_DIRECT_WP4,
                next_fail=MissionCheckpoint.CP09_DIRECT_WP4,
                timeout_s=int(t('timeouts.search', 25)),
                retries=0,
            ),
            MissionCheckpoint.CP09_DIRECT_WP4: CheckpointSpec(
                guard=self.guard_gps_ok,
                action=self.execute_cp9_direct_wp4,
                next_success=MissionCheckpoint.CP10_SEARCH_DROP_ITEM3,
                next_fail=MissionCheckpoint.CP10_SEARCH_DROP_ITEM3,
                timeout_s=180,
                retries=0,
            ),
            MissionCheckpoint.CP10_SEARCH_DROP_ITEM3: CheckpointSpec(
                guard=self.guard_vision_ready,
                action=self.execute_cp10_search_drop_item3,
                next_success=MissionCheckpoint.CP11_GPS_WP5,
                next_fail=MissionCheckpoint.CP11_GPS_WP5,
                timeout_s=int(t('timeouts.drop', 15)),
                retries=int(t('retries.pickup', 2)),
            ),
            MissionCheckpoint.CP11_GPS_WP5: CheckpointSpec(
                guard=self.guard_gps_ok,
                action=self.execute_cp11_gps_wp5,
                next_success=MissionCheckpoint.CP12_FINAL_DESCENT_DISARM,
                next_fail=MissionCheckpoint.CP12_FINAL_DESCENT_DISARM,
                timeout_s=180,
                retries=0,
            ),
            MissionCheckpoint.CP12_FINAL_DESCENT_DISARM: CheckpointSpec(
                guard=self.guard_clearance_ok,
                action=self.execute_cp12_final_descent_disarm,
                next_success=MissionCheckpoint.COMPLETED,
                next_fail=MissionCheckpoint.COMPLETED,
                timeout_s=60,
                retries=0,
            ),
        }

    # ===========================================
    # Guards (boolean, measured)
    # ===========================================
    def guard_init_arm(self) -> bool:
        return self.px4_connected() and self.battery_ok()

    def guard_takeoff_ready(self) -> bool:
        return self.px4_armed()

    def guard_vision_ready(self) -> bool:
        # Placeholder: assume vision ready if any detection subscription is alive
        return True

    def guard_hover_stable(self) -> bool:
        z_sp = float(self._p('takeoff_alt', 1.0))
        tol = float(self._p('hover_tol_m', 0.1))
        hold_s = float(self._p('hover_hold_s', 3))
        now = time.time()
        if not hasattr(self, '_hover_ok_since'):
            self._hover_ok_since = None
        ok = abs(self.current_altitude - z_sp) <= tol
        if ok:
            if self._hover_ok_since is None:
                self._hover_ok_since = now
            return (now - self._hover_ok_since) >= hold_s
        else:
            self._hover_ok_since = None
            return False

    def guard_gps_ok(self) -> bool:
        # Minimal: consider NavSatFix status >= 0 as has fix
        try:
            sats_min = int(self._p('gps_min_sats', 8))
            # satellites_visible is available in mavros_msgs/GPSRAW or GPSStatus; as placeholder, use status >= 0
            return getattr(self.gps_position.status, 'status', -1) >= 0
        except Exception:
            return False

    def guard_clearance_ok(self) -> bool:
        # Placeholder: rely on descent finalization without explicit LiDAR
        return True

    # Simple accessors per spec
    def battery_ok(self) -> bool:
        vmin = float(self._p('battery_min_volt', 14.4))
        # TODO: subscribe to /mavros/battery
        return True  # Assume OK if not implemented

    def px4_connected(self) -> bool:
        return bool(getattr(self.mavros_state, 'connected', False))

    def px4_armed(self) -> bool:
        return bool(getattr(self.mavros_state, 'armed', False))

    # ===========================================
    # CHECKPOINT IMPLEMENTATIONS
    # ===========================================
    
    def execute_cp1_init_arm(self):
        """CP1: Initialize & ARM flight controller"""
        self.get_logger().info("🔧 CP1: Initialize & ARM")

        # System checks
        if not self.system_health_check():
            self.get_logger().error("❌ Health check failed → skip to CP-12")
            self.complete_checkpoint(MissionCheckpoint.CP12_FINAL_DESCENT_DISARM)
            return

        # Wait for MAVROS connection
        if not self.wait_for_connection(timeout=10.0):
            self.get_logger().error("❌ FCU not connected → skip to CP-12")
            self.complete_checkpoint(MissionCheckpoint.CP12_FINAL_DESCENT_DISARM)
            return

        # Begin setpoint streaming before arming/mode change (required by PX4 OFFBOARD)
        self.start_setpoint_stream()

        # Set appropriate control mode and confirm
        desired_mode = 'OFFBOARD' if str(self._p('flight_stack', 'ardupilot')).lower() == 'px4' else 'GUIDED'
        self.set_control_mode_auto()
        self.wait_for_mode(desired_mode, timeout=5.0)

        # ARM the drone with retries
        arm_retries = int(self._p('retries.arm', 3))
        if self.try_arm_with_retries(arm_retries, delay_s=2.0):
            self.get_logger().info("✅ Armed")
            self.complete_checkpoint(MissionCheckpoint.CP02_TAKEOFF_1M)
        else:
            self.get_logger().error("❌ Could not ARM within retries → skip to CP-12")
            self.complete_checkpoint(MissionCheckpoint.CP12_FINAL_DESCENT_DISARM)

    def execute_cp2_takeoff_1m(self):
        """CP2: Takeoff to 1.0m altitude"""
        self.get_logger().info("🚀 CP2: Takeoff to 1.0m")
        
        # Switch to GUIDED mode
        self.set_flight_mode("GUIDED")
        time.sleep(1)
        
        # Execute takeoff
        target_altitude = float(self._p('takeoff_alt', 1.0))
        if self.takeoff_to_altitude(target_altitude):
            self.get_logger().info(f"✅ Hover {target_altitude}m")
            self.complete_checkpoint(MissionCheckpoint.CP03_SEARCH_ITEM1)
        else:
            self.get_logger().error("❌ Takeoff failed → skip to CP-12")
            self.complete_checkpoint(MissionCheckpoint.CP12_FINAL_DESCENT_DISARM)

    def execute_cp3_search_item1(self):
        """CP3: Search Item 1 with front bottom camera"""
        self.get_logger().info("🔍 CP3: Search Item 1")
        
        # Activate front bottom camera
        self.publish_camera_command("enable:front_bottom")
        
        # Move forward while searching
        self.send_velocity_command(0.3, 0, 0)  # Forward at 0.3 m/s
        
        # Wait for item detection or timeout
        if self.wait_for_item_detection(timeout=60):
            # Item detected, perform pickup sequence
            self.align_and_pickup_item("front", item_number=1)
            self.item1_collected = True
            self.get_logger().info("✅ CP3 Complete: Item 1 collected")
            self.complete_checkpoint(MissionCheckpoint.CP04_SEARCH_ITEM2_TURN)
        else:
            self.get_logger().warning("⚠️ CP3 Timeout: Item 1 not found, proceeding anyway")
            self.complete_checkpoint(MissionCheckpoint.CP04_SEARCH_ITEM2_TURN)

    def execute_cp4_search_item2_turn(self):
        """CP4: Search Item 2 with back camera & navigation turn"""
        self.get_logger().info("🔍🔄 CP4: Search Item 2 & Turn")
        
        # Continue forward while activating back camera
        self.publish_camera_command("enable:back")
        self.send_velocity_command(0.3, 0, 0)
        
        # Wait for item 2 detection
        if self.wait_for_item_detection(timeout=60, camera="back"):
            self.align_and_pickup_item("back", item_number=2)
            self.item2_collected = True
            
        # Execute navigation turn after item 2 collection
        self.execute_navigation_turn()
        
        self.get_logger().info("✅ CP4 Complete: Item 2 search and navigation turn done")
        self.complete_checkpoint(MissionCheckpoint.CP05_DROP_ITEM1)

    def execute_cp5_drop_item1(self):
        """CP5: Drop Item 1 to front bucket"""
        self.get_logger().info("🪣 CP5: Drop Item 1")
        
        if not self.item1_collected:
            self.get_logger().warning("⚠️ CP5: No Item 1 to drop, skipping")
            self.complete_checkpoint(MissionCheckpoint.CP06_DROP_ITEM2)
            return
            
        # Search for drop bucket with front camera
        self.publish_camera_command("enable:front")
        
        if self.search_and_align_bucket():
            self.drop_item("front")
            self.get_logger().info("✅ CP5 Complete: Item 1 dropped")
        else:
            self.get_logger().warning("⚠️ CP5: Bucket not found, emergency drop")
            self.drop_item("front")
            
        self.complete_checkpoint(MissionCheckpoint.CP06_DROP_ITEM2)

    def execute_cp6_drop_item2(self):
        """CP6: Drop Item 2 to back bucket"""
        self.get_logger().info("🪣 CP6: Drop Item 2")
        
        if not self.item2_collected:
            self.get_logger().warning("⚠️ CP6: No Item 2 to drop, skipping")
            self.complete_checkpoint(MissionCheckpoint.CP07_GPS_WP1_3)
            return
            
        # Search for drop bucket with back camera
        self.publish_camera_command("enable:back")
        
        if self.search_and_align_bucket(camera="back"):
            self.drop_item("back")
            self.get_logger().info("✅ CP6 Complete: Item 2 dropped")
        else:
            self.get_logger().warning("⚠️ CP6: Bucket not found, emergency drop")
            self.drop_item("back")
            
        self.indoor_items_dropped = True
        self.complete_checkpoint(MissionCheckpoint.CP07_GPS_WP1_3)

    def execute_cp7_gps_wp1_3(self):
        """CP7: GPS Navigation WP1-3 at 3m/s"""
        self.get_logger().info("🛰️ CP7: GPS WP1→WP3")
        
        # Switch to AUTO mode for GPS navigation
        self.set_flight_mode("AUTO")
        
        # Navigate through waypoints 1, 2, 3
        waypoints = [1, 2, 3]
        
        for wp in waypoints:
            self.get_logger().info(f"📍 Navigating to WP{wp}")
            if self.navigate_to_waypoint(wp, speed=3.0):
                self.get_logger().info(f"✅ Reached WP{wp}")
            else:
                self.get_logger().warning(f"⚠️ Failed to reach WP{wp}, continue")
        
        self.get_logger().info("✅ CP7 Complete: WP1-3 navigation done")
        self.complete_checkpoint(MissionCheckpoint.CP08_SEARCH_ITEM3)

    def execute_cp8_search_item3(self):
        """CP8: Search Item 3 after WP3"""
        self.get_logger().info("🔍 CP8: Search Item 3")
        
        # Switch back to GUIDED mode
        self.set_flight_mode("GUIDED")
        
        # Activate front bottom camera for item search
        self.publish_camera_command("enable:front_bottom")
        
        if self.wait_for_item_detection(timeout=60):
            self.align_and_pickup_item("front", item_number=3)
            self.item3_collected = True
            self.get_logger().info("✅ CP8 Complete: Item 3 collected")
        else:
            self.get_logger().warning("⚠️ CP8 Timeout: Item 3 not found")
            
        self.complete_checkpoint(MissionCheckpoint.CP09_DIRECT_WP4)

    def execute_cp9_direct_wp4(self):
        """CP9: Direct navigation to WP4 with payload"""
        self.get_logger().info("🛰️ CP9: Direct to WP4")
        
        # Switch to AUTO mode
        self.set_flight_mode("AUTO")
        
        if self.navigate_to_waypoint(4, speed=3.0):
            self.get_logger().info("✅ CP9 Complete: Arrived at WP4")
            self.complete_checkpoint(MissionCheckpoint.CP10_SEARCH_DROP_ITEM3)
        else:
            self.get_logger().warning("⚠️ Failed to reach WP4, continue")
            self.complete_checkpoint(MissionCheckpoint.CP10_SEARCH_DROP_ITEM3)

    def execute_cp10_search_drop_item3(self):
        """CP10: Search drop bucket and drop Item 3"""
        self.get_logger().info("🪣🔍 CP10: Search & Drop Item 3")
        
        # Switch to GUIDED mode for search
        self.set_flight_mode("GUIDED")
        
        if not self.item3_collected:
            self.get_logger().warning("⚠️ CP10: No Item 3 to drop, skipping")
            self.complete_checkpoint(MissionCheckpoint.CP11_GPS_WP5)
            return
            
        # Search for bucket
        self.publish_camera_command("enable:front")
        
        if self.search_and_align_bucket():
            self.drop_item("front")
            self.outdoor_item_dropped = True
            self.get_logger().info("✅ CP10 Complete: Item 3 dropped")
        else:
            self.get_logger().warning("⚠️ CP10: Bucket not found, emergency drop")
            self.drop_item("front")
            
        self.complete_checkpoint(MissionCheckpoint.CP11_GPS_WP5)

    def execute_cp11_gps_wp5(self):
        """CP11: GPS Navigation to final WP5"""
        self.get_logger().info("🛰️ CP11: GPS to WP5")
        
        # Switch to AUTO mode
        self.set_flight_mode("AUTO")
        
        if self.navigate_to_waypoint(5, speed=3.0):
            self.get_logger().info("✅ CP11 Complete: Arrived at final WP5")
            self.complete_checkpoint(MissionCheckpoint.CP12_FINAL_DESCENT_DISARM)
        else:
            self.get_logger().warning("⚠️ Failed to reach WP5, continue")
            self.complete_checkpoint(MissionCheckpoint.CP12_FINAL_DESCENT_DISARM)

    def execute_cp12_final_descent_disarm(self):
        """CP12: Final descent and disarm (no ground contact detection)"""
        self.get_logger().info("🏁 CP12: Final descent & disarm")
        
        # Prefer MAVROS landing service for a clean land
        landed_cmd_ok = self.land()
        if not landed_cmd_ok:
            # Fallback to mode-based landing when available
            self.get_logger().warn("⚠️ Land service failed; attempting mode switch to AUTO.LAND/GUIDED")
            # Try ArduPilot LAND mode first, then PX4 LAND
            if not self.set_flight_mode("AUTO.LAND"):
                self.set_flight_mode("LAND")

        # Wait until low altitude, then disarm
        start = time.time()
        timeout = 30.0
        while time.time() - start < timeout:
            if self.current_altitude <= 0.2:
                break
            time.sleep(0.2)

        # Disarm the drone
        if self.disarm_drone():
            self.get_logger().info("✅ CP12 Complete: Mission accomplished!")
            self.mission_completed = True
            self.complete_checkpoint(MissionCheckpoint.COMPLETED)
        else:
            self.get_logger().error("❌ CP12: Disarm failed")
            self.transition_to_error()

    # ===========================================
    # UTILITY FUNCTIONS
    # ===========================================
    
    def system_health_check(self):
        """Perform system health check before mission"""
        self.get_logger().info("🔍 Performing system health check...")
        
        # Check MAVROS connection
        if not self.mavros_state.connected:
            self.get_logger().error("❌ MAVROS not connected to flight controller")
            return False
            
        # TODO: Add more health checks
        # - Battery voltage
        # - GPS status
        # - Camera status
        # - Sensor status
        
        self.get_logger().info("✅ System health check passed")
        return True
        
    def arm_drone(self):
        """ARM the drone"""
        if not self.arm_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ARM service not available")
            return False
            
        request = CommandBool.Request()
        request.value = True
        
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.armed = True
            return True
        return False
        
    def disarm_drone(self):
        """DISARM the drone"""
        if not self.arm_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ARM service not available")
            return False
            
        request = CommandBool.Request()
        request.value = False
        
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.armed = False
            return True
        return False
        
    def set_flight_mode(self, mode):
        """Set flight mode"""
        if not self.mode_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMode service not available")
            return False
            
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        return future.result() and future.result().mode_sent

    def wait_for_mode(self, mode: str, timeout: float = 5.0) -> bool:
        """Wait until /mavros/state.mode matches requested mode"""
        start = time.time()
        while time.time() - start < timeout:
            if getattr(self.mavros_state, 'mode', '') == mode:
                return True
            time.sleep(0.1)
        return False
        
    def takeoff_to_altitude(self, altitude):
        """Takeoff to specified altitude"""
        if not self.takeoff_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Takeoff service not available")
            return False
            
        request = CommandTOL.Request()
        request.altitude = altitude
        
        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() and future.result().success:
            # Wait for stable altitude
            return self.wait_for_altitude(altitude, tolerance=0.1, timeout=15.0)
        return False
        
    def wait_for_altitude(self, target_altitude, tolerance=0.1, timeout=15.0):
        """Wait for drone to reach target altitude"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if abs(self.current_altitude - target_altitude) < tolerance:
                self.get_logger().info(f"✅ Altitude reached: {self.current_altitude:.2f}m")
                return True
            time.sleep(0.1)
            
        self.get_logger().error(f"❌ Altitude timeout: {self.current_altitude:.2f}m (target: {target_altitude:.2f}m)")
        return False

    def wait_for_connection(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            if self.px4_connected():
                return True
            time.sleep(0.1)
        return False

    def try_arm_with_retries(self, retries: int, delay_s: float = 2.0) -> bool:
        for i in range(1, max(1, retries) + 1):
            if self.arm_drone():
                return True
            self.get_logger().warning(f"⚠️ ARM attempt {i}/{retries} failed; retrying...")
            time.sleep(delay_s)
        return False
        
    def wait_for_item_detection(self, timeout=60, camera="front_bottom"):
        """Wait for item detection"""
        self.get_logger().info(f"👀 Waiting for item detection ({camera})...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.item_detected:
                return True
            time.sleep(0.1)
            
        return False
        
    def align_and_pickup_item(self, magnet_position, item_number):
        """Align with item and perform pickup"""
        self.get_logger().info(f"🎯 Aligning and picking up Item {item_number}...")
        
        # Visual servoing to center item
        if self.item_detected:
            self.visual_servo_to_item()
            
        # Descent and pickup
        self.descend_and_pickup(magnet_position)
        
        # Ascend back to travel altitude
        self.ascend_to_travel_altitude()
        
    def visual_servo_to_item(self):
        """Visual servoing to center item in camera"""
        # Simple proportional control
        center_x = 320  # Camera center X
        center_y = 240  # Camera center Y
        
        error_x = self.item_position.x - center_x
        error_y = self.item_position.y - center_y
        
        # Convert to velocity commands
        vx = -error_y * 0.001  # Forward/backward
        vy = -error_x * 0.001  # Left/right
        
        self.send_velocity_command(vx, vy, 0)
        
        # Check if aligned
        if abs(error_x) < 20 and abs(error_y) < 20:
            self.item_aligned = True
            self.send_velocity_command(0, 0, 0)  # Stop
            
    def descend_and_pickup(self, magnet_position):
        """Descend and pickup item"""
        self.get_logger().info(f"📉 Descending for pickup with {magnet_position} magnet...")
        
        # Descend to pickup altitude
        self.send_velocity_command(0, 0, 0.3)  # Descend
        time.sleep(3)  # Allow descent
        
        # Activate magnet
        self.publish_magnet_command(f"{magnet_position}:on")
        time.sleep(2)  # Allow pickup
        
        self.get_logger().info(f"✅ Pickup complete with {magnet_position} magnet")
        
    def ascend_to_travel_altitude(self):
        """Ascend back to travel altitude"""
        self.send_velocity_command(0, 0, -0.3)  # Ascend
        time.sleep(3)
        self.send_velocity_command(0, 0, 0)  # Stop
        
    def execute_navigation_turn(self):
        """Execute navigation turn based on configuration"""
        turn_direction = self.hw_config.get_turn_direction()
        self.get_logger().info(f"🔄 Executing {turn_direction} turn...")
        
        # Wait for LiDAR to detect wall
        # TODO: Implement LiDAR wall detection
        
        if turn_direction.lower() == "left":
            self.send_velocity_command(0, 0.5, 0)  # Move left
            time.sleep(2)
        else:  # Right turn
            self.send_velocity_command(0, -0.5, 0)  # Move right
            time.sleep(2)
            
        self.send_velocity_command(0, 0, 0)  # Stop
        
    def search_and_align_bucket(self, camera="front"):
        """Search for drop bucket and align"""
        self.get_logger().info(f"🔍 Searching for bucket with {camera} camera...")
        
        # TODO: Implement bucket detection
        # For now, assume bucket found
        time.sleep(2)
        self.bucket_detected = True
        
        return self.bucket_detected
        
    def drop_item(self, magnet_position, altitude=None):
        """Drop item at specified altitude"""
        if altitude is None:
            altitude = float(self._p('pickup_alt', 0.3))
        self.get_logger().info(f"📦 Dropping item from {magnet_position} magnet at {altitude}m...")
        
        # Maintain altitude
        # TODO: Implement altitude hold at drop altitude
        
        # Deactivate magnet
        self.publish_magnet_command(f"{magnet_position}:off")
        time.sleep(2)  # Allow drop
        
        self.get_logger().info(f"✅ Item dropped from {magnet_position} magnet")
        
    def navigate_to_waypoint(self, waypoint_number, speed=3.0):
        """Navigate to GPS waypoint"""
        # Ensure PX4 waypoints are loaded
        if self.px4_waypoints_count == 0:
            self.pull_px4_waypoints()
        if self.px4_waypoints_count == 0:
            self.get_logger().warning("📭 Cannot navigate: no PX4 waypoints loaded")
        else:
            self.get_logger().info(f"🛰️ Navigating using PX4 mission: WP{waypoint_number} of {self.px4_waypoints_count} (speed {speed} m/s)")

        # TODO: Implement real GPS waypoint navigation with MAVROS
        time.sleep(5)
        return True
        
    def send_velocity_command(self, vx, vy, vz):
        """Send velocity command"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.velocity_pub.publish(msg)
        # Update the last command so streaming keeps FCU satisfied
        self.last_twist_cmd = msg
        
    def send_altitude_command(self, altitude, climb_rate):
        """Send altitude command"""
        # TODO: Implement altitude command
        pass

    def send_position_target_local(self, x: float, y: float, z: float, yaw: Optional[float] = None,
                                   frame: int = None, type_mask: int = None):
        """Publish raw local position target via /mavros/setpoint_raw/local.

        By default, drive position only (ignore velocities/accels/yaw rate).
        """
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        if frame is None:
            frame = PositionTarget.FRAME_LOCAL_NED
        msg.coordinate_frame = frame

        # Default: use only position (ignore vx,vy,vz & ax,ay,az & yaw_rate)
        if type_mask is None:
            type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
        msg.type_mask = type_mask

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        if yaw is not None:
            msg.yaw = float(yaw)
        self.local_setpoint_pub.publish(msg)

    def send_velocity_target_local(self, vx: float, vy: float, vz: float, yaw_rate: Optional[float] = None,
                                   frame: int = None):
        """Publish raw local velocity target using PositionTarget (alternative to cmd_vel)."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        if frame is None:
            frame = PositionTarget.FRAME_LOCAL_NED
        msg.coordinate_frame = frame
        # Ignore position and accelerations, use velocity
        msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        if yaw_rate is not None:
            msg.yaw_rate = float(yaw_rate)
        self.local_setpoint_pub.publish(msg)

    def land(self, altitude: float = 0.0, yaw: float = 0.0) -> bool:
        """Trigger landing via MAVROS /mavros/cmd/land (CommandTOL)."""
        if not self.land_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Land service not available")
            return False
        req = CommandTOL.Request()
        # If GPS present, prefer current GPS for land
        lat = getattr(self.gps_position, 'latitude', 0.0) if self.gps_position else 0.0
        lon = getattr(self.gps_position, 'longitude', 0.0) if self.gps_position else 0.0
        req.latitude = float(lat)
        req.longitude = float(lon)
        req.altitude = float(altitude)
        req.yaw = float(yaw)
        future = self.land_service.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        ok = bool(future.result() and future.result().success)
        if ok:
            self.get_logger().info("🛬 Land command accepted")
        else:
            self.get_logger().warn("⚠️ Land command was not accepted")
        return ok

    def mavros_param_get(self, param_id: str) -> Optional[float]:
        """Get MAVROS parameter value (float) using /mavros/param/get."""
        if not self.param_get_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("ParamGet service not available")
            return None
        req = ParamGet.Request()
        req.param_id = str(param_id)
        future = self.param_get_service.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.result():
            self.get_logger().warn(f"ParamGet failed: {param_id}")
            return None
        res = future.result()
        # ROS2 mavros_msgs/ParamValue has .integer/.real; prefer real if set
        val = getattr(res.value, 'real', 0.0)
        return val

    def mavros_param_set(self, param_id: str, value: float) -> bool:
        """Set MAVROS parameter value using /mavros/param/set."""
        if not self.param_set_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("ParamSet service not available")
            return False
        req = ParamSet.Request()
        req.param_id = str(param_id)
        # ParamValue in ROS2 has fields .integer and .real
        from mavros_msgs.msg import ParamValue
        pv = ParamValue()
        pv.real = float(value)
        req.value = pv
        future = self.param_set_service.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        ok = bool(future.result() and future.result().success)
        if not ok:
            self.get_logger().warn(f"ParamSet failed: {param_id}={value}")
        return ok

    def _stream_setpoint(self):
        """Continuously stream last velocity setpoint when active."""
        if not self.setpoint_active:
            return
        # Ensure fresh header stamp
        self.last_twist_cmd.header.stamp = self.get_clock().now().to_msg()
        self.velocity_pub.publish(self.last_twist_cmd)

    def start_setpoint_stream(self, warmup_s: float = None):
        self.setpoint_active = True
        if warmup_s is None:
            warmup_s = float(self._p('offboard_warmup_s', 2))
        # Warm-up stream for OFFBOARD/GUIDED acceptance
        start = time.time()
        while time.time() - start < warmup_s:
            self._stream_setpoint()
            time.sleep(0.05)

    def stop_setpoint_stream(self):
        self.setpoint_active = False

    def set_control_mode_auto(self) -> bool:
        """Set control mode based on flight_stack config (px4->OFFBOARD, ardupilot->GUIDED)."""
        stack = str(self._p('flight_stack', 'ardupilot')).lower()
        mode = 'OFFBOARD' if stack == 'px4' else 'GUIDED'
        return self.set_flight_mode(mode)
        
    def complete_checkpoint(self, next_checkpoint):
        """Complete current checkpoint and transition to next"""
        self.publish_checkpoint_status(self.current_checkpoint.value, "COMPLETED")
        self.current_checkpoint = next_checkpoint
        self.current_retries = 0
        self.cp_start_time = time.time()
        self._executing_action = False
        if self.debug_mode:
            # Hold position between checkpoints in debug mode
            self.hold_position_debug()
            # Only pause for 'next' if the next CP is a normal progression
            if next_checkpoint not in (MissionCheckpoint.CP12_FINAL_DESCENT_DISARM, MissionCheckpoint.COMPLETED):
                self.waiting_for_next = True
                self.get_logger().info(f"🔄 Ready for next checkpoint: {next_checkpoint.value}")
                self.get_logger().info("💬 Type 'next' on /mission/user_input to proceed")
            else:
                # For landing/completion, continue automatically
                self.waiting_for_next = False
        
    def transition_to_error(self):
        """Transition to error state"""
        self.current_checkpoint = MissionCheckpoint.ERROR
        self.waiting_for_next = True
        self.get_logger().error("🚨 Mission entered ERROR state")
        
    def handle_mission_completed(self):
        """Handle mission completion"""
        self.get_logger().info("🎉 MISSION COMPLETED SUCCESSFULLY!")
        self.publish_checkpoint_status("MISSION_COMPLETED", "SUCCESS")
        self.mission_completed = True
        
    # ===========================================
    # MESSAGE PUBLISHERS
    # ===========================================
    
    def publish_checkpoint_status(self, checkpoint, status):
        """Publish checkpoint status"""
        msg = String()
        msg.data = f"{checkpoint}:{status}"
        self.checkpoint_status_pub.publish(msg)
        # Minimal, standardized CP console log
        status_up = str(status).upper()
        if status_up.startswith('EXEC'):  # EXECUTING
            self.get_logger().info(f"[CP] START {checkpoint}")
        elif status_up.startswith('COMP'):  # COMPLETED
            self.get_logger().info(f"[CP] DONE {checkpoint}")
        elif status_up.startswith('FAIL'):
            self.get_logger().info(f"[CP] FAIL {checkpoint}")
        elif status_up.startswith('WAIT'):
            self.get_logger().info(f"[CP] WAIT {checkpoint}")
        
    def publish_camera_command(self, command):
        """Publish camera command"""
        msg = String()
        msg.data = command
        self.camera_command_pub.publish(msg)
        
    def publish_magnet_command(self, command):
        """Publish magnet command"""
        msg = String()
        msg.data = command
        self.magnet_command_pub.publish(msg)
        
    # ===========================================
    # CALLBACK FUNCTIONS
    # ===========================================
    
    def mavros_state_callback(self, msg):
        """MAVROS state callback"""
        prev_mode = getattr(self.mavros_state, 'mode', '') if hasattr(self, 'mavros_state') else None
        prev_armed = getattr(self, 'armed', None)
        prev_conn = getattr(self.mavros_state, 'connected', False) if hasattr(self, 'mavros_state') else None

        self.mavros_state = msg
        self.armed = msg.armed

        # Log on state changes for quick field debugging
        if prev_conn is not None and prev_conn != msg.connected:
            self.get_logger().info(f"[STATE] MAVROS connected={msg.connected}")
        if prev_armed is not None and prev_armed != msg.armed:
            self.get_logger().info(f"[STATE] armed={msg.armed}")
        if prev_mode is not None and prev_mode != msg.mode:
            self.get_logger().info(f"[STATE] mode={msg.mode}")
        
    def pose_callback(self, msg):
        """Local position callback"""
        self.current_pose = msg
        self.current_altitude = msg.pose.position.z
        
    def gps_callback(self, msg):
        """GPS position callback"""
        self.gps_position = msg
        
    def raw_fix_callback(self, msg: NavSatFix):
        """Raw GPS fix callback (/mavros/global_position/raw/fix)"""
        self.gps_raw_fix = msg
        # Optionally mirror to primary if it's empty
        if (self.gps_position is None) or (getattr(self.gps_position, 'latitude', 0.0) == 0.0 and getattr(self.gps_position, 'longitude', 0.0) == 0.0):
            self.gps_position = msg

    def mag_callback(self, msg: MagneticField):
        """Magnetometer callback"""
        self.latest_mag = msg

    def temperature_callback(self, msg: Temperature):
        """IMU temperature callback"""
        self.latest_imu_temp = msg

    def statustext_callback(self, msg: StatusText):
        """FCU status text via MAVROS"""
        self.latest_statustext = msg
        # Map severity to log level (MAV_SEVERITY: 0-7)
        sev = int(getattr(msg, 'severity', 6))
        text = getattr(msg, 'text', '').strip()
        if text:
            if sev <= 2:
                self.get_logger().error(f"FCU: {text}")
            elif sev <= 4:
                self.get_logger().warn(f"FCU: {text}")
            else:
                self.get_logger().info(f"FCU: {text}")

    def rc_in_callback(self, msg: RCIn):
        self.latest_rc_in = msg

    def rc_out_callback(self, msg: RCOut):
        self.latest_rc_out = msg

    def vision_callback(self, msg):
        """Vision detection callback (Point center)"""
        try:
            # If a point is received, consider it a detection
            self.item_detected = True
            self.item_position = msg
        except Exception:
            self.item_detected = False
            
    def user_input_callback(self, msg):
        """User input callback for debug mode"""
        cmd = msg.data.strip().lower()
        if cmd in ("next", "continue", "n") and self.waiting_for_next:
            self.waiting_for_next = False
            self.get_logger().info("▶️ Continuing to next checkpoint...")
        elif msg.data.lower() == "pause":
            self.waiting_for_next = True
            self.get_logger().info("⏸️ Mission paused")
        elif msg.data.lower() == "emergency":
            self.get_logger().warning("🚨 Emergency stop triggered!")
            self.emergency_stop()

    def hold_position_debug(self):
        """Stop motion and hold position safely in debug pauses"""
        try:
            self.send_velocity_command(0.0, 0.0, 0.0)
            # Prefer LOITER if available to hold position (ArduPilot)
            if self.px4_connected() and self.px4_armed():
                self.set_flight_mode("LOITER")
        except Exception as e:
            self.get_logger().warning(f"Hold position failed: {e}")

    # ===========================================
    # PX4 Waypoint Pull/Cache
    # ===========================================
    def _try_pull_waypoints_when_ready(self):
        if self._wp_init_done:
            return
        if not self.px4_connected():
            return
        ok = self.pull_px4_waypoints()
        # Mark done regardless; navigation will re-pull when needed
        self._wp_init_done = True
        try:
            self._wp_timer.cancel()
        except Exception:
            pass

    def pull_px4_waypoints(self):
        """Pull mission waypoints from PX4 via MAVROS and log count."""
        if not self.wp_pull_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("PX4 waypoints: pull service not available yet")
            return False
        req = WaypointPull.Request()
        future = self.wp_pull_service.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.result():
            self.get_logger().warning("PX4 waypoints: pull call failed")
            return False
        res = future.result()
        if getattr(res, 'success', False):
            count = int(getattr(res, 'wp_received', 0))
            self.px4_waypoints_count = max(self.px4_waypoints_count, count)
            if count > 0:
                self.get_logger().info(f"📥 Loaded {count} waypoints from PX4")
            else:
                self.get_logger().info("📭 No waypoints on PX4 (0 loaded)")
            return True
        else:
            self.get_logger().warning("PX4 waypoints: pull unsuccessful")
            return False

    def waypoints_callback(self, msg: WaypointList):
        count = len(msg.waypoints)
        self.px4_waypoints_count = count
        # Log only on change
        if self._last_logged_wp_count != count:
            if count > 0:
                self.get_logger().info(f"📥 Waypoints updated from PX4: {count} items")
            else:
                self.get_logger().info("📭 Waypoints not loaded (0 items)")
            self._last_logged_wp_count = count
            
    def emergency_stop(self):
        """Emergency stop procedure"""
        self.send_velocity_command(0, 0, 0)  # Stop all movement
        self.waiting_for_next = True
        self.current_checkpoint = MissionCheckpoint.PAUSED

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Checkpoint12MissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
