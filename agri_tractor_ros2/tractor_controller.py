#!/usr/bin/env python3
"""tractor_controller : /cmd_vel -> vitesse d'essieu + force sur le guidon.

Remplace l'ancien nœud à base de subprocess. 100 % topics ROS 2, pontés vers
Gazebo par ros_gz_bridge (config/bridge.yaml).

Entrées
  /cmd_vel   geometry_msgs/Twist   linear.x (m/s) et angular.z (rad/s) voulus
  /odom      nav_msgs/Odometry     orientation (repère monde) + vitesse de lacet mesurée
Sorties
  /wheel_cmd        std_msgs/Float64                 vitesse de l'essieu rigide (rad/s)
  /apply_force      ros_gz_interfaces/EntityWrench   -> /world/agri_world/wrench/persistent
  /apply_force/clear ros_gz_interfaces/Entity        -> /world/agri_world/wrench/clear
  /handle_force     geometry_msgs/WrenchStamped      force guidon (repère guidon_link) pour RViz/plot

Pourquoi « persistent » + « clear » ? Le topic /wrench simple n'agit que pendant
UN pas physique (1 ms) : publié à 20 Hz, la force serait appliquée 2 % du temps.
La version persistante reste active jusqu'à effacement ; on la rafraîchit à
chaque cycle (la direction monde change quand le tracteur tourne) en effaçant
d'abord l'ancienne pour ne jamais en cumuler deux.
"""
import math

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench
from std_msgs.msg import Float64

from agri_tractor_ros2.handle_math import (YawRateController, clamp,
                                           handle_wrench_world,
                                           torque_to_handle_force)


class TractorController(Node):

    def __init__(self):
        super().__init__('tractor_controller')
        self.declare_parameters('', [
            ('wheel_radius', 0.25),
            ('max_wheel_speed', 8.0),               # rad/s (~2 m/s)
            ('handle_position', [-1.0, 0.0, 0.65]),  # guidon dans base_link (m)
            ('max_handle_force', 400.0),            # N, effort humain réaliste
            ('k_ff', 750.0),                        # N.m par rad/s (anticipation)
            ('kp', 150.0),                          # N.m par rad/s d'erreur
            ('ki', 80.0),                           # N.m par rad
            ('i_limit', 3.0),                       # borne de l'intégrale (rad)
            ('closed_loop', True),
            ('cmd_timeout', 0.5),                   # s sans /cmd_vel -> arrêt
            ('rate', 20.0),                         # Hz
            ('clear_gap', 0.015),                   # s entre clear et nouvelle force
            ('clear_before_set', True),
            ('force_deadband', 2.0),                # N
            ('link_name', 'tractor::base_link'),
        ])
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.r_wheel = gp('wheel_radius')
        self.max_wheel = gp('max_wheel_speed')
        self.handle_position = tuple(float(v) for v in gp('handle_position'))
        self.f_max = gp('max_handle_force')
        self.timeout = gp('cmd_timeout')
        self.dt = 1.0 / gp('rate')
        self.clear_first = gp('clear_before_set')
        self.deadband = gp('force_deadband')
        self.link_name = gp('link_name')
        self.ctrl = YawRateController(gp('k_ff'), gp('kp'), gp('ki'),
                                      gp('i_limit'), gp('closed_loop'))

        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd_time = None
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.w_meas = 0.0
        self.have_odom = False
        self.force_active = False
        self.last_force = 0.0
        self.pending = None

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub_wheel = self.create_publisher(Float64, 'wheel_cmd', 10)
        self.pub_wrench = self.create_publisher(EntityWrench, 'apply_force', 10)
        self.pub_clear = self.create_publisher(Entity, 'apply_force/clear', 10)
        self.pub_dbg = self.create_publisher(WrenchStamped, 'handle_force', 10)

        self.timer = self.create_timer(self.dt, self.loop)
        self.set_timer = self.create_timer(gp('clear_gap'), self.send_pending)
        self.set_timer.cancel()

        if self.handle_position[0] >= 0.0:
            self.get_logger().warn('handle_position.x >= 0 : le guidon devrait être à l\'arrière (X-)')
        self.get_logger().info(
            f'Prêt : guidon={self.handle_position}, Fmax={self.f_max} N, '
            f'{"boucle fermée" if self.ctrl.closed_loop else "boucle ouverte"}')

    # ------------------------------------------------------------ callbacks
    def on_cmd(self, msg):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.orientation = (q.x, q.y, q.z, q.w)
        self.w_meas = msg.twist.twist.angular.z
        self.have_odom = True

    # ------------------------------------------------------------ boucle
    def loop(self):
        now = self.get_clock().now()
        stale = (self.last_cmd_time is None or
                 (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout)
        v, w = (0.0, 0.0) if stale else (self.v_cmd, self.w_cmd)

        # 1) Essieu rigide : une seule vitesse pour les deux roues
        wheel = clamp(v / self.r_wheel, -self.max_wheel, self.max_wheel)
        self.pub_wheel.publish(Float64(data=wheel))

        # 2) Guidon : couple de lacet voulu -> force latérale au guidon
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            self.ctrl.reset()
            f = 0.0
        else:
            sat = 0
            if abs(self.last_force) >= self.f_max - 1e-6:
                # signe de l'erreur qui augmenterait encore la saturation
                sat = int(math.copysign(1.0, self.last_force * self.handle_position[0]))
            w_meas = self.w_meas if self.have_odom else w
            tau = self.ctrl.update(w, w_meas, self.dt, sat)
            f = clamp(torque_to_handle_force(tau, self.handle_position[0]), -self.f_max, self.f_max)
        self.last_force = f
        self.apply(f, now)

    def apply(self, f, now):
        dbg = WrenchStamped()
        dbg.header.stamp = now.to_msg()
        dbg.header.frame_id = 'guidon_link'
        dbg.wrench.force.y = f
        self.pub_dbg.publish(dbg)

        if abs(f) < self.deadband:
            if self.force_active:
                self.pub_clear.publish(self.entity())
                self.force_active = False
            return

        wr = handle_wrench_world(f, self.handle_position, self.orientation)
        msg = EntityWrench()
        msg.header.stamp = now.to_msg()
        msg.entity = self.entity()
        (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z) = wr.force
        (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z) = wr.torque

        if self.clear_first and self.force_active:
            self.pub_clear.publish(self.entity())
            self.pending = msg
            self.set_timer.reset()      # la nouvelle force part clear_gap plus tard
        else:
            self.pub_wrench.publish(msg)
        self.force_active = True

    def send_pending(self):
        self.set_timer.cancel()
        if self.pending is not None:
            self.pub_wrench.publish(self.pending)
            self.pending = None

    def entity(self):
        e = Entity()
        e.name = self.link_name
        e.type = Entity.LINK
        return e

    def stop(self):
        self.pub_wheel.publish(Float64(data=0.0))
        self.pub_clear.publish(self.entity())


def main(args=None):
    rclpy.init(args=args)
    node = TractorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001  (contexte déjà fermé)
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
