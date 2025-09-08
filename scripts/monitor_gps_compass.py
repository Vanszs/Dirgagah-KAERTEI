#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import NavSatFix, MagneticField
from std_msgs.msg import Float64, Int32


class GpsCompassMonitor(Node):
    def __init__(self):
        super().__init__('gps_compass_monitor')
        self.fix = None  # type: NavSatFix | None
        self.hdg = None  # type: float | None
        self.mag = None  # type: MagneticField | None
        self.sats = None  # type: int | None

        self.create_subscription(NavSatFix, '/mavros/global_position/global', self._on_fix, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self._on_fix, 10)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self._on_hdg, 10)
        self.create_subscription(MagneticField, '/mavros/imu/mag', self._on_mag, 10)
        self.create_subscription(Int32, '/mavros/global_position/raw/satellites', self._on_sats, 10)

        self.timer = self.create_timer(1.0, self._tick)
        self.start_time = Clock().now()
        self.get_logger().info('GPS/Compass monitor started (1 Hz)')

    def _on_fix(self, msg: NavSatFix):
        self.fix = msg

    def _on_hdg(self, msg: Float64):
        self.hdg = float(msg.data)

    def _on_mag(self, msg: MagneticField):
        self.mag = msg

    def _on_sats(self, msg: Int32):
        self.sats = int(msg.data)

    def _fix_status_text(self, status: int) -> str:
        # sensor_msgs/NavSatStatus
        mapping = {
            -1: 'NO_FIX',
             0: 'FIX',
             1: 'SBAS_FIX',
             2: 'GBAS_FIX',
        }
        return mapping.get(status, str(status))

    def _tick(self):
        stamp = Clock().now().to_msg()
        header = f"[{stamp.sec}.{str(stamp.nanosec//1000000).zfill(3)}] "

        # GPS
        if self.fix is not None:
            fs = self.fix
            status_text = self._fix_status_text(getattr(fs.status, 'status', -1))
            sats = self.sats if self.sats is not None else 'n/a'
            self.get_logger().info(
                header + (
                    f"GPS lat={fs.latitude:.7f} lon={fs.longitude:.7f} alt={fs.altitude:.2f}m "
                    f"status={status_text} sats={sats}"
                )
            )
        else:
            self.get_logger().warn(header + 'GPS: no data yet')

        # Compass heading
        if self.hdg is not None:
            self.get_logger().info(header + f"HDG={self.hdg:.1f}°")
        else:
            self.get_logger().warn(header + 'HDG: no data yet')

        # Magnetometer
        if self.mag is not None:
            m = self.mag.magnetic_field
            self.get_logger().info(header + f"MAG x={m.x:.3f} y={m.y:.3f} z={m.z:.3f} (Tesla)")
        else:
            self.get_logger().warn(header + 'MAG: no data yet')


def main():
    rclpy.init()
    node = GpsCompassMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

