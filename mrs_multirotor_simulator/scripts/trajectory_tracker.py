#!/usr/bin/env python3
import math
import csv
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from mrs_msgs.msg import HwApiControlGroupCmd


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle)) # <-pi, pi>


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class PIDController:
    def __init__(self, kp=0.0, kd=0.0, ki=0.0, saturation=-1.0, antiwindup=-1.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.saturation = saturation
        self.antiwindup = antiwindup
        self.last_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        difference = (error - self.last_error) / dt
        self.last_error = error

        val = self.kp * error + self.kd * difference + self.ki * self.integral

        if self.saturation > 0.0:
            val = max(-self.saturation, min(self.saturation, val))

        if self.antiwindup > 0.0 and abs(val) < self.antiwindup:
            self.integral += error * dt

        return val


class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        # Subscribers and publishers
        self.odom_sub = self.create_subscription(
            Odometry, '/multirotor_simulator/uav1/odom', self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            HwApiControlGroupCmd, '/multirotor_simulator/uav1/control_group_cmd', 10
        )
        self.teleport_pub = self.create_publisher(
            Odometry, '/multirotor_simulator/uav1/teleport_cmd', 10
        )
        self.path_pub = self.create_publisher(
            Path, '/multirotor_simulator/uav1/planned_path', 10
        )

        # UAV states
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        # start synchronization
        self.odom_received = False
        self.teleport_sent = False
        self.has_teleported = False

        # PID regulators
        self.pid_lon = PIDController(kp=0.20, kd=0.05, ki=0.00, saturation=10.0, antiwindup=5.0)
        self.pid_lat = PIDController(kp=0.01, kd=0.01, ki=0.001, saturation=0.35, antiwindup=0.2)
        self.pid_alt = PIDController(kp=0.12, kd=0.04, ki=0.005, saturation=0.35, antiwindup=0.2)
        self.kp_yaw = 0.40  

        # data loading
        script_dir = os.path.dirname(os.path.abspath(__file__))
        flight_csv = os.path.join(script_dir, 'flight_data.csv')
        actions_csv = os.path.join(script_dir, 'actions_data.csv')

        self.trajectory_points = []
        self.load_data(flight_csv, actions_csv)
        self.current_target_index = 0

        # Časovače: 50 Hz řízení (0.02 s), 1 Hz vykreslování v RVizu
        self.timer = self.create_timer(0.02, self.control_loop)
        self.viz_timer = self.create_timer(1.0, self.publish_path)

    def load_data(self, flight_file, actions_file):
        try:
            with open(flight_file, 'r') as ff, open(actions_file, 'r') as af:
                f_rows = list(csv.reader(ff))[1:]
                a_rows = list(csv.reader(af))[1:]

                num_rows = min(len(f_rows), len(a_rows))
                for i in range(num_rows):
                    f_row = f_rows[i]
                    a_row = a_rows[min(i + 1, len(a_rows) - 1)]

                    self.trajectory_points.append({
                        'x': float(f_row[0]),
                        'y': float(f_row[1]),
                        'z': float(f_row[2]),
                        'vel': float(f_row[3]),
                        'yaw': float(f_row[4]),
                        'pitch': float(f_row[5]),
                        'roll': float(f_row[6]),
                        'time': float(f_row[7]),
                        'ff_throttle': float(a_row[0]),
                        'ff_roll': math.radians(float(a_row[1])),
                        'ff_pitch': math.radians(float(a_row[2])),
                    })
            self.get_logger().info(f"Načteno {len(self.trajectory_points)} bodů (trajektorie + FF akce).")
        except Exception as e:
            self.get_logger().error(f"Chyba při čtení CSV: {e}")

    def teleport_to_start(self):
        pt0 = self.trajectory_points[0]
        msg = Odometry()
        msg.pose.pose.position.x = pt0['x']
        msg.pose.pose.position.y = pt0['y']
        msg.pose.pose.position.z = pt0['z']
        msg.pose.pose.orientation = quaternion_from_euler(pt0['roll'], pt0['pitch'], pt0['yaw'])

        v0 = pt0['vel']
        msg.twist.twist.linear.x = v0 * math.cos(pt0['yaw']) * math.cos(pt0['pitch'])
        msg.twist.twist.linear.y = v0 * math.sin(pt0['yaw']) * math.cos(pt0['pitch'])
        msg.twist.twist.linear.z = v0 * math.sin(pt0['pitch'])

        self.teleport_pub.publish(msg)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def control_loop(self):
        if not self.trajectory_points:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # telelport na start a synchronizace
        if not self.has_teleported:
            if self.teleport_pub.get_subscription_count() == 0 or not self.odom_received:
                return

            pt0 = self.trajectory_points[0]
            dist_xy = math.hypot(self.current_x - pt0['x'], self.current_y - pt0['y'])
            dist_z = abs(self.current_z - pt0['z'])

            if self.teleport_sent and dist_xy < 1.0 and dist_z < 1.0:
                self.has_teleported = True
                self.trajectory_start_time = current_time
                self.last_time = current_time
                self.pid_lon.reset()
                self.pid_lat.reset()
                self.pid_alt.reset()
                self.get_logger().info("Teleport done")
            else:
                if not hasattr(self, 'last_tp_time') or (current_time - self.last_tp_time) > 0.2:
                    self.teleport_to_start()
                    self.teleport_sent = True
                    self.last_tp_time = current_time
            return

        # dt for PID
        dt = current_time - self.last_time
        if dt <= 0.0:
            return
        self.last_time = current_time

        elapsed_time = current_time - self.trajectory_start_time
        cmd = HwApiControlGroupCmd()
        cmd.stamp = self.get_clock().now().to_msg()

        # end of trajectory
        if elapsed_time > self.trajectory_points[-1]['time']:
            if not hasattr(self, 'finished_logged'):
                self.get_logger().info("Konec trajektorie.")
                self.finished_logged = True
            cmd.throttle, cmd.pitch, cmd.roll, cmd.yaw = 30.0, 0.0, 0.0, 0.0
            self.cmd_pub.publish(cmd)
            return

        # actual segment, alpha = 0-1 where I am between two trajectory points (p0, p1)
        while (self.current_target_index < len(self.trajectory_points) - 2 and
               elapsed_time >= self.trajectory_points[self.current_target_index + 1]['time']):
            self.current_target_index += 1

        p0 = self.trajectory_points[self.current_target_index]
        p1 = self.trajectory_points[self.current_target_index + 1]
        dt_seg = p1['time'] - p0['time']
        alpha = max(0.0, min(1.0, (elapsed_time - p0['time']) / dt_seg)) if dt_seg > 0.0 else 0.0

        # Plynulá interpolace referenčního stavu pro zpětnou vazbu
        target_x = p0['x'] + alpha * (p1['x'] - p0['x'])
        target_y = p0['y'] + alpha * (p1['y'] - p0['y'])
        target_z = p0['z'] + alpha * (p1['z'] - p0['z'])
        target_yaw = wrap_angle(p0['yaw'] + alpha * wrap_angle(p1['yaw'] - p0['yaw']))

        # Výpočet chyb v souřadnicovém systému dráhy
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        error_z = target_z - self.current_z

        e_lon = dx * math.cos(target_yaw) + dy * math.sin(target_yaw)
        e_lat = -dx * math.sin(target_yaw) + dy * math.cos(target_yaw)
        e_yaw = wrap_angle(target_yaw - self.current_yaw)

        # Výpočet akčních zásahů PID regulátorů
        u_lon = self.pid_lon.update(e_lon, dt)
        u_lat = self.pid_lat.update(e_lat, dt) + self.kp_yaw * e_yaw
        u_alt = self.pid_alt.update(error_z, dt)

        # feed forward + PID
        cmd.throttle = p0['ff_throttle'] + u_lon
        cmd.roll = p0['ff_roll'] + u_lat
        cmd.pitch = p0['ff_pitch'] + u_alt
        cmd.yaw = 0.0

        self.cmd_pub.publish(cmd)

        # Výpis diagnostiky 
        # if not hasattr(self, 'last_log_time') or (current_time - self.last_log_time) >= 1.0:
        #     error_xy = math.hypot(dx, dy)
        #     self.get_logger().info(
        #         f"t={elapsed_time:4.1f}s | Z -> Cíl: {target_z:.2f} m, Real: {self.current_z:.2f} m "
        #         f"(Chyba Z: {error_z:+.2f} m) | Chyba XY: {error_xy:.2f} m"
        #     )
        #     self.last_log_time = current_time

    def publish_path(self):
        if not self.trajectory_points:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'simulator_origin'

        for pt in self.trajectory_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = pt['x']
            pose.pose.position.y = pt['y']
            pose.pose.position.z = pt['z']
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main():
    rclpy.init()
    node = TrajectoryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()