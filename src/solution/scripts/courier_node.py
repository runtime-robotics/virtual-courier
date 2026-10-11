#!/usr/bin/env python3
"""Заготовка решения «Виртуальный курьер».

Здесь уже подключены все топики робота. Ваша задача — заменить демонстрационное
поведение в control_loop() своим алгоритмом: найти путь к активным станциям,
забрать грузы и выгрузить их в зоне выгрузки у Старта.

ВХОД (симуляция → решение)
    /scan                 sensor_msgs/LaserScan    лидар 360°, 0.12–8 м, 10 Гц
    /odom                 nav_msgs/Odometry        одометрия по энкодерам (с дрейфом!), 50 Гц
    /tf, /tf_static       odom→base_link, base_link→laser/camera_link/imu_link/tray_link
    /imu                  sensor_msgs/Imu          ориентация, угл. скорость, ускорение, 50 Гц
    /camera/image_raw     sensor_msgs/Image        640×480 bgra8, 10 Гц
    /camera/camera_info   sensor_msgs/CameraInfo
    /courier/cargo_count  std_msgs/UInt8           сколько грузов лежит на лотке (весовой датчик)
    /courier/tray_state   std_msgs/Float64         текущий угол лотка, рад
    /clock                время симуляции (use_sim_time:=true)

ВЫХОД (решение → симуляция)
    /cmd_vel              geometry_msgs/Twist      linear.x ≤ 0.7 м/с, angular.z ≤ 3 рад/с
    /courier/tray_cmd     std_msgs/Float64         угол наклона лотка 0 … 1.05 рад

ЗАПРЕЩЕНО: подписываться на /referee/*, использовать любые данные о «истинной» позе.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float64, UInt8

TRAY_DOWN = 0.0
TRAY_UNLOAD = 1.0   # рад — при таком наклоне груз соскальзывает назад

# ArUco-маркеры полигона (словарь DICT_4X4_50):
#   0 — стена зоны выгрузки у Старта, 1/2/3 — станции A/B/C.
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_NAMES = {0: 'ФИНИШ', 1: 'станция A', 2: 'станция B', 3: 'станция C'}


class Courier(Node):
    def __init__(self):
        super().__init__('courier')
        self.declare_parameter('gui', True)
        self.gui = self.get_parameter('gui').value

        # ---- последние данные с датчиков
        self.scan = None
        self.pose = (0.0, 0.0, 0.0)     # x, y, yaw в системе odom
        self.speed = (0.0, 0.0)         # v, w
        self.imu = None
        self.image = None               # BGR numpy-массив
        self.cargo_count = 0
        self.tray_angle = 0.0
        self.bridge = CvBridge()

        # ---- подписки
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/courier/cargo_count', self.on_cargo, 10)
        self.create_subscription(Float64, '/courier/tray_state', self.on_tray, 10)

        # ---- публикаторы
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tray_pub = self.create_publisher(Float64, '/courier/tray_cmd', 10)

        self.state = 'WAIT_SENSORS'
        self.timer = self.create_timer(0.1, self.control_loop)   # 10 Гц по времени симуляции
        self.get_logger().info('Courier запущен, ждём данные датчиков…')

    # ================================================================ колбэки

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)
        self.speed = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_imu(self, msg):
        self.imu = msg

    def on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def on_cargo(self, msg):
        if msg.data != self.cargo_count:
            self.get_logger().info(f'грузов на лотке: {msg.data}')
        self.cargo_count = msg.data

    def on_tray(self, msg):
        self.tray_angle = msg.data

    # ================================================================ действия

    def drive(self, v, w):
        """Задать линейную (м/с) и угловую (рад/с) скорость."""
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.drive(0.0, 0.0)

    def set_tray(self, angle):
        """Наклонить лоток: 0 — горизонтально, ~1.0 рад — выгрузка назад."""
        self.tray_pub.publish(Float64(data=float(angle)))

    # ================================================================ восприятие

    def sector_min(self, from_deg, to_deg):
        """Минимальная дальность лидара в секторе углов [from, to] (0° — вперёд, + — влево)."""
        s = self.scan
        if s is None:
            return float('inf')
        best = float('inf')
        n = len(s.ranges)
        for deg in range(int(from_deg), int(to_deg) + 1):
            i = int(round((math.radians(deg) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[i]
            if s.range_min <= r < best:
                best = r
        return best

    def detect_markers(self):
        """Найти ArUco-маркеры на кадре. Возвращает [(id, cx, cy, площадь_px)]."""
        if self.image is None:
            return []
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT)
        found = []
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if int(i) not in MARKER_NAMES:   # ложные срабатывания на текстурах
                    continue
                pts = c.reshape(4, 2)
                cx, cy = pts.mean(axis=0)
                found.append((int(i), float(cx), float(cy), float(cv2.contourArea(pts))))
        return found

    # ================================================================ логика

    def control_loop(self):
        if self.state == 'WAIT_SENSORS':
            if self.scan is not None:
                self.get_logger().info('датчики готовы — поехали')
                self.set_tray(TRAY_DOWN)
                self.state = 'EXPLORE'
            return

        # ------------------------------------------------------------------
        # ДЕМО-ПОВЕДЕНИЕ: простой объезд препятствий по лидару.
        # TODO: замените своим алгоритмом, например:
        #   1. построить карту (slam_toolbox) или исследовать лабиринт правилом руки;
        #   2. найти станции (цветная разметка пола, ArUco 1–3, лампа: зелёная = активна);
        #   3. остановиться в зоне станции на ≥ 2 с → груз упадёт на лоток (см. cargo_count);
        #   4. довезти, не уронив, до зоны выгрузки у Старта (ArUco 0), встать к стене кормой;
        #   5. set_tray(TRAY_UNLOAD), подождать, set_tray(TRAY_DOWN).
        # ------------------------------------------------------------------
        if self.state == 'EXPLORE':
            front = self.sector_min(-25, 25)
            left = self.sector_min(30, 90)
            right = self.sector_min(-90, -30)
            if front < 0.45:
                self.drive(0.0, 1.2 if left > right else -1.2)
            else:
                # держимся подальше от ближайшей боковой стены
                steer = max(-0.8, min(0.8, 0.8 * (min(left, 2.0) - min(right, 2.0))))
                self.drive(0.35, steer)

        markers = self.detect_markers()
        for mid, cx, _, area in markers:
            self.get_logger().info(
                f'вижу маркер {mid} ({MARKER_NAMES.get(mid, "?")}), x={cx:.0f}px, S={area:.0f}px²',
                throttle_duration_sec=2.0)

        if self.gui and self.image is not None:
            view = self.image.copy()
            for mid, cx, cy, _ in markers:
                cv2.circle(view, (int(cx), int(cy)), 8, (0, 255, 0), 2)
                cv2.putText(view, MARKER_NAMES.get(mid, str(mid)), (int(cx) + 10, int(cy)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(view, f'state={self.state} cargo={self.cargo_count}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('courier camera', view)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = Courier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
