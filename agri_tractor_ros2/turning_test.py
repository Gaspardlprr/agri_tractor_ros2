#!/usr/bin/env python3
"""turning_test : essai de braquage automatique pour le calibrage (Étape 3).

Envoie un /cmd_vel constant pendant `duration` secondes, enregistre /odom, puis
affiche :
  - le rayon de braquage mesuré (cercle ajusté sur la trajectoire)
  - la vitesse de lacet obtenue vs demandée
  - la position du point de pivot sur l'axe du robot (doit être proche de
    l'éperon, x = -0.45 m, si la physique est réaliste)
  - la force guidon moyenne

Exemples :
  ros2 run agri_tractor_ros2 turning_test --ros-args -p v:=0.5 -p w:=0.4
  ros2 run agri_tractor_ros2 turning_test --ros-args -p v:=0.0 -p w:=0.5   # pivot sur place
Un CSV est écrit dans /tmp/turning_test_<v>_<w>.csv pour tracer la trajectoire.
"""
import math
import statistics

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from agri_tractor_ros2.handle_math import fit_circle, pivot_point_x, yaw_from_quat


class TurningTest(Node):

    def __init__(self):
        super().__init__('turning_test')
        self.declare_parameters('', [
            ('v', 0.5), ('w', 0.4), ('duration', 15.0), ('settle', 3.0),
            ('eperon_x', -0.45)])
        self.v = self.get_parameter('v').value
        self.w = self.get_parameter('w').value
        self.duration = self.get_parameter('duration').value
        self.settle = self.get_parameter('settle').value
        self.eperon_x = self.get_parameter('eperon_x').value

        self.samples = []   # (t, x, y, yaw, vx_body, vy_body, wz)
        self.forces = []
        self.t0 = None
        self.done = False
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        self.create_subscription(WrenchStamped, 'handle_force', self.on_force, 50)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Essai : v={self.v} m/s, w={self.w} rad/s, {self.duration} s')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        if self.t0 is None:
            return
        t = self.now() - self.t0
        p = m.pose.pose
        yaw = yaw_from_quat((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        tw = m.twist.twist   # OdometryPublisher : twist exprimé dans base_link
        self.samples.append((t, p.position.x, p.position.y, yaw,
                             tw.linear.x, tw.linear.y, tw.angular.z))

    def on_force(self, m):
        if self.t0 is not None and self.now() - self.t0 > self.settle:
            self.forces.append(m.wrench.force.y)

    def tick(self):
        if self.done:
            return
        if self.t0 is None:
            self.t0 = self.now()
        t = self.now() - self.t0
        cmd = Twist()
        if t < self.duration:
            cmd.linear.x, cmd.angular.z = self.v, self.w
            self.pub.publish(cmd)
        else:
            self.pub.publish(cmd)  # arrêt
            self.done = True
            self.report()

    def report(self):
        s = [x for x in self.samples if x[0] > self.settle]
        log = self.get_logger()
        if len(s) < 10:
            log.error('Pas assez de données /odom : la simulation tourne-t-elle ?')
            return
        wz = [x[6] for x in s]
        vx = [x[4] for x in s]
        vy = [x[5] for x in s]
        w_mean = statistics.fmean(wz)
        lines = [
            '================ RÉSULTAT ESSAI DE BRAQUAGE ================',
            f'commande         : v = {self.v:+.3f} m/s   w = {self.w:+.3f} rad/s',
            f'mesuré           : v = {statistics.fmean(vx):+.3f} m/s   w = {w_mean:+.3f} rad/s '
            f'(suivi {100 * w_mean / self.w if abs(self.w) > 1e-6 else float("nan"):.0f} %)',
            f'glissement lat.  : vy moyen au CdM = {statistics.fmean(vy):+.3f} m/s',
        ]
        px = pivot_point_x(statistics.fmean(vy), w_mean)
        if px is not None:
            lines.append(f'point de pivot   : x = {px:+.2f} m (éperon à {self.eperon_x:+.2f} m)'
                         + ('  OK' if abs(px - self.eperon_x) < 0.25 else '  -> à régler'))
        try:
            cx, cy, r = fit_circle([(x[1], x[2]) for x in s])
            lines.append(f'rayon de braquage: R = {r:.2f} m (centre {cx:+.2f}, {cy:+.2f})')
        except ValueError:
            lines.append('rayon de braquage: trajectoire rectiligne (pas de cercle)')
        if self.forces:
            lines.append(f'force guidon     : moy {statistics.fmean(self.forces):+.0f} N, '
                         f'max |F| {max(abs(f) for f in self.forces):.0f} N')
        lines.append('============================================================')
        for ln in lines:
            log.info(ln)
        path = f'/tmp/turning_test_{self.v:+.2f}_{self.w:+.2f}.csv'
        with open(path, 'w', encoding='utf-8') as f:
            f.write('t,x,y,yaw,vx,vy,wz\n')
            for x in self.samples:
                f.write(','.join(f'{v:.4f}' for v in x) + '\n')
        log.info(f'Trajectoire : {path}')
        if math.isfinite(w_mean) and abs(self.w) > 1e-6 and abs(w_mean) < 0.3 * abs(self.w):
            log.warn('Le tracteur tourne trop peu : augmenter max_handle_force ou k_ff, '
                     'ou baisser wheel_mu_lat (URDF).')


def main(args=None):
    rclpy.init(args=args)
    node = TurningTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
