#!/usr/bin/env python3
import math
import signal
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float64, UInt8

from navigation import GridMap, cell_of, wrap


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


class Courier(Node):
    def __init__(self):
        super().__init__('courier')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.declare_parameter('gui', False)
        self.scan = self.image = self.camera_matrix = self.raw_odom = None
        self.imu_origin = self.imu_yaw = None
        self.imu_stamp = self.imu_rate = 0.
        self.pose_history = deque(maxlen=250)
        self.scan_pose = np.zeros(3)
        self.pose = np.zeros(3)
        self.odom_ready = False
        self.sensor_time = {'scan': -math.inf, 'odom': -math.inf, 'imu': -math.inf}
        self.scan_wall_time = 0.
        self.cargo_count = 0
        self.tray_angle = 0.
        self.bridge = CvBridge()
        self.map = GridMap()
        self.visited = set()
        self.stations = {}
        self.serviced = set()
        self.home_normal = None
        self.current_cell = (0, 0)
        self.target_cell = None
        self.goal = None
        self.path = []
        self.state = 'WAIT_SENSORS'
        self.since = 0.
        self.started = None
        self.last_map = self.last_plan = self.last_report = -math.inf
        self.command = np.zeros(2)
        self.returning = False
        self.blocked_since = None
        self.progress_pose = np.zeros(2)
        self.progress_time = 0.
        self.aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tray_pub = self.create_publisher(Float64, '/courier/tray_cmd', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.path_pub = self.create_publisher(Path, '/plan', 1)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, msg_type, callback, qos in (
            ('/scan', LaserScan, self.on_scan, sensor_qos),
            ('/odom', Odometry, self.on_odom, 1),
            ('/imu', Imu, self.on_imu, sensor_qos),
            ('/camera/image_raw', Image, self.on_image, sensor_qos),
            ('/camera/camera_info', CameraInfo, self.on_camera_info, sensor_qos),
            ('/courier/cargo_count', UInt8, self.on_cargo, 10),
            ('/courier/tray_state', Float64, self.on_tray, 10),
        ):
            self.create_subscription(msg_type, topic, callback, qos)
        self.create_timer(.1, self.control_loop)
        self.get_logger().info('Waiting for lidar, odometry, IMU and camera')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def transition(self, state):
        self.state, self.since = state, self.now()
        self.progress_pose = self.pose[:2].copy()
        self.progress_time = self.since
        self.get_logger().info(f'{state} cell={self.current_cell} cargo={self.cargo_count}')

    def on_scan(self, msg):
        self.scan = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        self.sensor_time['scan'] = stamp
        if self.pose_history:
            self.scan_pose = min(self.pose_history, key=lambda p: abs(p[0]-stamp))[1].copy()
        self.scan_wall_time = time.monotonic()

    def on_imu(self, msg):
        yaw = yaw_of(msg.orientation)
        if not math.isfinite(yaw):
            return
        if self.imu_origin is None:
            # The rules guarantee an initial heading along a grid axis.
            self.imu_origin = round(yaw / (math.pi/2)) * math.pi/2
        self.imu_yaw = wrap(yaw-self.imu_origin)
        self.imu_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        self.imu_rate = msg.angular_velocity.z
        self.sensor_time['imu'] = self.imu_stamp

    def on_odom(self, msg):
        p = msg.pose.pose
        raw = np.array([p.position.x, p.position.y, yaw_of(p.orientation)])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        heading = self.imu_yaw
        if heading is not None:
            heading = wrap(heading + self.imu_rate*np.clip(stamp-self.imu_stamp, -.1, .1))
        if not np.all(np.isfinite(raw)):
            return
        if self.raw_odom is not None and self.imu_yaw is not None:
            delta = raw[:2] - self.raw_odom[:2]
            correction = heading - raw[2]
            c, s = math.cos(correction), math.sin(correction)
            self.pose[:2] += (c*delta[0]-s*delta[1], s*delta[0]+c*delta[1])
        self.raw_odom = raw
        if self.imu_yaw is not None:
            self.pose[2] = heading
        self.pose_history.append((stamp, self.pose.copy()))
        self.odom_ready = True
        self.sensor_time['odom'] = stamp

    def on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def on_camera_info(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def on_cargo(self, msg):
        if msg.data != self.cargo_count:
            self.get_logger().info(f'Cargo sensor: {self.cargo_count} -> {msg.data}')
        self.cargo_count = msg.data

    def on_tray(self, msg):
        self.tray_angle = msg.data

    def drive(self, v=0., w=0., emergency=False):
        desired = np.array([v, w])
        self.command += np.clip(desired-self.command, [-.06, -.22], [.045, .22])
        if emergency:
            self.command[:] = 0.
        msg = Twist()
        msg.linear.x, msg.angular.z = map(float, self.command)
        self.cmd_pub.publish(msg)

    def set_tray(self, angle):
        self.tray_pub.publish(Float64(data=float(angle)))

    def observe(self):
        if self.image is None or self.camera_matrix is None:
            return
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco)
        if ids is None:
            return
        s = .09
        square = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
        yaw = self.pose[2]
        c, sn = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -sn], [sn, c]])
        for points, mid in zip(corners, ids.ravel()):
            mid = int(mid)
            if mid not in (0, 1, 2, 3) or cv2.contourArea(points) < 70:
                continue
            ok, rvec, tvec = cv2.solvePnP(square, points.reshape(4, 2), self.camera_matrix,
                                         np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or not .2 < tvec[2, 0] < 4.0:
                continue
            normal = cv2.Rodrigues(rvec)[0][:, 2]
            forward = np.array([tvec[2, 0]+.178, -tvec[0, 0]])
            inward = np.array([normal[2], -normal[0]])
            if np.dot(inward, forward) > 0:
                inward = -inward
            wall = self.pose[:2] + rotation @ forward
            inward = rotation @ inward
            axis = int(np.argmax(np.abs(inward)))
            cardinal = np.zeros(2)
            cardinal[axis] = math.copysign(1., inward[axis])
            cell = cell_of(*(wall + .973*cardinal))
            if mid == 0 and cell == (0, 0):
                if self.home_normal is None:
                    self.get_logger().info(f'Home marker found, inward={cardinal}')
                self.home_normal = cardinal
            elif mid and max(abs(cell[0]), abs(cell[1])) <= 7:
                if mid not in self.stations:
                    self.get_logger().info(f'Marker {mid}: station cell {cell}')
                self.stations[mid] = cell

    def floor_station(self):
        if self.image is None:
            return None
        h, w = self.image.shape[:2]
        hsv = cv2.cvtColor(self.image[int(.72*h):, int(.25*w):int(.75*w)], cv2.COLOR_BGR2HSV)
        for mid, lo, hi in ((1, 18, 38), (2, 78, 105), (3, 135, 170)):
            mask = (hsv[:, :, 0] >= lo) & (hsv[:, :, 0] <= hi) & (hsv[:, :, 1] > 85) & (hsv[:, :, 2] > 35)
            if np.mean(mask) > .45:
                return mid
        return None

    def update_map(self):
        scan = self.scan
        angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
        ranges = np.array(scan.ranges)
        self.map.integrate(self.scan_pose, angles, ranges, scan.range_max)
        self.map.update_graph()
        self.observe()
        if self.now()-self.last_report > 2:
            self.publish_map()
            self.get_logger().info(f'pose={np.round(self.pose, 2)} state={self.state} '
                                   f'visited={len(self.visited)} known={len(self.map.graph)} goal={np.round(self.goal, 2) if self.goal is not None else None} cmd={np.round(self.command, 2)}')
            self.last_report = self.now()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.map.resolution
        msg.info.width = msg.info.height = self.map.size
        msg.info.origin.position.x = msg.info.origin.position.y = self.map.origin
        msg.info.origin.orientation.w = 1.
        grid = np.full(self.map.evidence.shape, -1, np.int8)
        grid[self.map.evidence < -.2] = 0
        grid[self.map.evidence > .6] = 100
        msg.data = grid.ravel().tolist()
        self.map_pub.publish(msg)

    def choose_goal(self):
        home = self.map.route(self.current_cell, {(0, 0)})
        remaining = 590 - (self.now()-self.started)
        if self.cargo_count >= 3 or len(self.serviced) >= 3 or \
                (self.cargo_count and remaining < len(home)*8+40):
            if not self.returning:
                self.get_logger().info(f'Returning home: {remaining:.0f}s left, {max(0, len(home)-1)} edges')
            self.returning = True
        if self.returning:
            route = home
        else:
            unserved = {cell for mid, cell in self.stations.items() if mid not in self.serviced}
            route = self.map.route(self.current_cell, unserved-self.visited)
            if not route:
                route = self.map.route(self.current_cell, set(self.map.graph)-self.visited)
            if not route:
                self.returning = True
                route = home
        if self.returning and self.current_cell == (0, 0):
            self.transition('FIND_HOME')
            return
        if len(route) < 2:
            self.transition('LOOK')
            return
        self.target_cell = route[1]
        self.goal = self.map.cell_goal(self.target_cell)
        self.path = []
        self.last_plan = -math.inf
        self.transition('MOVE')

    def navigate(self):
        distance = float(np.linalg.norm(self.goal-self.pose[:2]))
        if distance < .11:
            self.drive()
            return True
        now = self.now()
        if np.linalg.norm(self.pose[:2]-self.progress_pose) > .12:
            self.progress_pose = self.pose[:2].copy()
            self.progress_time = now
        if now-self.progress_time > 8:
            self.drive(emergency=True)
            self.recovery_state = self.state
            self.transition('RECOVER')
            return False
        if now-self.last_plan > .6:
            if self.state == 'MOVE':
                self.goal = self.map.cell_goal(self.target_cell, self.goal)
            self.path = self.map.plan(self.pose[:2], self.goal)
            self.last_plan = now
            msg = Path()
            msg.header.frame_id = 'odom'
            msg.header.stamp = self.get_clock().now().to_msg()
            for x, y in self.path[::3]:
                p = PoseStamped()
                p.header = msg.header
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.w = 1.
                msg.poses.append(p)
            self.path_pub.publish(msg)
        if not self.path:
            self.drive()
            if self.blocked_since is None:
                self.blocked_since = now
            if now-self.blocked_since > 5 and self.state == 'MOVE':
                self.get_logger().warning(f'No local path to {self.target_cell}; retry another edge')
                self.map.block_edge(self.current_cell, self.target_cell)
                self.transition('ARRIVE')
                self.blocked_since = None
            return False
        self.blocked_since = None
        nearest = min(range(len(self.path)), key=lambda i: math.dist(self.path[i], self.pose[:2]))
        target = None
        for candidate in self.path[nearest+1:]:
            if math.dist(candidate, self.pose[:2]) > .4 or not self.map.line_clear(self.pose[:2], candidate):
                break
            target = candidate
        if target is None:
            self.drive(emergency=True)
            self.last_plan = -math.inf
            return False
        error = wrap(math.atan2(target[1]-self.pose[1], target[0]-self.pose[0])-self.pose[2])
        w = float(np.clip(3*error, -1.4, 1.4))
        speed = .62 if self.cargo_count else .65
        v = min(speed, distance*1.2)*max(0., math.cos(error)) if abs(error) < .18 else 0.
        angles = self.scan.angle_min + np.arange(len(self.scan.ranges))*self.scan.angle_increment
        ranges = np.array(self.scan.ranges)
        with np.errstate(invalid='ignore'):
            xs, ys = ranges*np.cos(angles)+.09, ranges*np.sin(angles)
        ahead = xs[(np.abs(ys) < .215) & (xs > 0) & np.isfinite(xs)]
        if len(ahead):
            clearance = float(np.min(ahead))-.215
            v = min(v, math.sqrt(max(0., 1.0*clearance)))
        self.drive(v, w)
        return False

    def control_loop(self):
        now = self.now()
        if self.state == 'WAIT_SENSORS':
            if self.scan is not None and self.odom_ready and self.imu_yaw is not None and self.image is not None:
                self.started = now
                self.set_tray(0.)
                self.transition('INITIAL_LOOK')
            return
        if self.state in ('DONE', 'UNLOAD_FAILED', 'TIME_LIMIT'):
            self.drive(emergency=True)
            return
        if now-self.started >= 600:
            self.transition('TIME_LIMIT')
            self.drive(emergency=True)
            return
        if any(now-t > .8 for t in self.sensor_time.values()) or time.monotonic()-self.scan_wall_time > 2:
            self.drive(emergency=True)
            return
        if now-self.last_map > .3:
            self.update_map()
            self.last_map = now
        if self.state == 'INITIAL_LOOK':
            self.drive(0., 1.25)
            if now-self.since > 5.8:
                self.visited.add((0, 0))
                self.drive()
                self.transition('ARRIVE')
        elif self.state == 'MOVE':
            if self.navigate():
                self.current_cell = self.target_cell
                self.visited.add(self.current_cell)
                self.transition('ARRIVE')
        elif self.state == 'RECOVER':
            angles = self.scan.angle_min + np.arange(len(self.scan.ranges))*self.scan.angle_increment
            ranges = np.asarray(self.scan.ranges)
            rear = ranges[(np.cos(angles) < -.85) & np.isfinite(ranges)]
            if len(rear) and np.min(rear) > .50:
                self.drive(-.10, 0.)
            else:
                self.drive()
            if now-self.since > 1.8:
                self.last_plan = -math.inf
                self.transition(self.recovery_state)
        elif self.state == 'ARRIVE':
            self.drive()
            mid = self.floor_station()
            if mid:
                self.stations[mid] = self.current_cell
            station = next((mid for mid, cell in self.stations.items()
                            if cell == self.current_cell and mid not in self.serviced), None)
            if now-self.since > (3.8 if station else .4):
                if station:
                    self.serviced.add(station)
                    self.get_logger().info(f'Station {station} inspected, cargo={self.cargo_count}')
                self.choose_goal()
        elif self.state == 'LOOK':
            self.drive(0., 1.)
            if now-self.since > 3:
                self.choose_goal()
        elif self.state == 'FIND_HOME':
            if self.home_normal is None:
                self.drive(0., .8)
            else:
                self.goal = -.55*self.home_normal
                self.path = []
                self.last_plan = -math.inf
                self.transition('DOCK')
        elif self.state == 'DOCK':
            if self.navigate():
                self.transition('ALIGN_UNLOAD')
        elif self.state == 'ALIGN_UNLOAD':
            error = wrap(math.atan2(self.home_normal[1], self.home_normal[0])-self.pose[2])
            self.drive(0., float(np.clip(2.5*error, -.7, .7)))
            if abs(error) < .035:
                self.drive(emergency=True)
                self.transition('UNLOAD')
        elif self.state == 'UNLOAD':
            self.drive(emergency=True)
            self.set_tray(1.0)
            if now-self.since > 5:
                self.set_tray(0.)
                self.transition('DONE' if not self.cargo_count else 'UNLOAD_FAILED')
        elif self.state in ('DONE', 'UNLOAD_FAILED'):
            self.drive(emergency=True)


def main():
    rclpy.init()
    node = Courier()
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive(emergency=True)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
