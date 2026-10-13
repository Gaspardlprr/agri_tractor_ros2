#!/usr/bin/env python3
"""vtr_node : Visual Teach & Repeat sur la caméra du motoculteur.

Modes (paramètre `mode`) :
  view    extraction ORB en temps réel, image annotée sur /vtr/debug_image
          et nombre de points sur /vtr/feature_count (1re brique de l'Étape 4)
  teach   idem + enregistrement des keyframes pendant que TU conduis au clavier
  repeat  rejoue le chemin appris : publie /cmd_vel (le tractor_controller
          s'occupe des roues et du guidon)

Abonnements : /camera/image_raw (sensor_msgs/Image), /odom (nav_msgs/Odometry),
              /cmd_vel (en teach, pour mémoriser la vitesse demandée)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String

from agri_tractor_ros2.handle_math import yaw_from_quat
from agri_tractor_ros2.vtr_core import (FeatureExtractor, Localizer, PathRecorder,
                                        draw_debug, focal_from_hfov, heading_error,
                                        image_to_gray, load_path,
                                        match, steering_command)


class VTRNode(Node):

    def __init__(self):
        super().__init__('vtr_node')
        self.declare_parameters('', [
            ('mode', 'view'),
            ('path_dir', '~/vtr_paths/path1'),
            ('n_features', 800),
            ('keyframe_dist', 0.30),
            ('keyframe_yaw', 0.17),
            ('hfov', 1.2),
            ('repeat_speed', 0.4),
            ('k_heading', 1.5),
            ('w_max', 0.6),
            ('search_window', 3),
            ('min_inliers', 25),
            ('lost_timeout', 2.0),
            ('rate', 10.0),
            ('publish_debug', True),
        ])
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mode = gp('mode')
        if self.mode not in ('view', 'teach', 'repeat'):
            raise ValueError("mode doit être 'view', 'teach' ou 'repeat'")
        self.hfov = gp('hfov')
        self.v_rep = gp('repeat_speed')
        self.k_heading = gp('k_heading')
        self.w_max = gp('w_max')
        self.min_inl = gp('min_inliers')
        self.lost_timeout = gp('lost_timeout')
        self.debug = gp('publish_debug')
        self.ext = FeatureExtractor(gp('n_features'))

        self.gray = None
        self.fx = None
        self.pose = None          # (x, y, yaw)
        self.w_meas = 0.0
        self.v_meas = 0.0
        self.s = 0.0
        self.last_xy = None
        self.lost_since = None
        self.finished = False

        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.pub_dbg = self.create_publisher(Image, 'vtr/debug_image', 2)
        self.pub_count = self.create_publisher(Int32, 'vtr/feature_count', 10)
        self.pub_state = self.create_publisher(String, 'vtr/state', 10)
        self.pub_err = self.create_publisher(Float32, 'vtr/heading_error', 10)

        if self.mode == 'teach':
            self.rec = PathRecorder(gp('path_dir'), gp('keyframe_dist'), gp('keyframe_yaw'))
            self.get_logger().info(f'TEACH : conduis au clavier, keyframes -> {self.rec.dir}')
        elif self.mode == 'repeat':
            d, kfs = load_path(gp('path_dir'))
            self.loc = Localizer(d, kfs, self.ext, gp('search_window'))
            self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
            self.get_logger().info(f'REPEAT : {len(kfs)} keyframes, {kfs[-1].s:.1f} m à parcourir')

        self.create_timer(1.0 / gp('rate'), self.step)

    # ------------------------------------------------------------ entrées
    def on_image(self, msg):
        try:
            self.gray = image_to_gray(msg.data, msg.width, msg.height, msg.step, msg.encoding)
            self.header = msg.header
            if self.fx is None:
                self.fx = focal_from_hfov(msg.width, self.hfov)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)

    def on_odom(self, msg):
        p = msg.pose.pose
        x, y = p.position.x, p.position.y
        yaw = yaw_from_quat((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        if self.last_xy is not None:
            self.s += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        self.pose = (x, y, yaw)
        self.w_meas = msg.twist.twist.angular.z
        self.v_meas = msg.twist.twist.linear.x
        if self.mode == 'teach':
            self.rec.update_odometry(x, y)

    # ------------------------------------------------------------ boucle
    def step(self):
        if self.gray is None:
            self.pub_state.publish(String(data='attente image'))
            if self.mode == 'repeat':
                self.pub_cmd.publish(Twist())
            return
        gray = self.gray
        kps, desc = self.ext.extract(gray)
        self.pub_count.publish(Int32(data=len(kps)))
        text = [f'{self.mode.upper()}  ORB: {len(kps)}']
        overlay = None

        if self.mode == 'teach':
            if self.pose is not None and self.rec.should_record(self.pose[2]):
                kf = self.rec.add(gray, *self.pose, self.w_meas, self.v_meas)
                self.get_logger().info(f'keyframe {kf.index}  s={kf.s:.2f} m')
            text.append(f'keyframes: {len(self.rec.keyframes)}  s={self.rec.s:.1f} m')

        elif self.mode == 'repeat':
            overlay = self.repeat_step(gray, kps, desc, text)

        self.pub_state.publish(String(data=' | '.join(text)))
        if self.debug and self.pub_dbg.get_subscription_count() > 0:
            self.publish_debug(draw_debug(gray, kps, text, overlay))

    def repeat_step(self, gray, kps, desc, text):
        cmd = Twist()
        if self.finished:
            self.pub_cmd.publish(cmd)
            text.append('TERMINÉ')
            return None
        if self.fx is None:
            self.pub_cmd.publish(cmd)
            return None
        kf, dx, n = self.loc.localize(kps, desc, self.s)
        now = self.get_clock().now().nanoseconds * 1e-9
        if kf is None or n < self.min_inl:
            # perdu : on ralentit puis on s'arrête
            self.lost_since = self.lost_since or now
            lost = now - self.lost_since
            if lost < self.lost_timeout:
                cmd.linear.x = 0.5 * self.v_rep
            text.append(f'PERDU {lost:.1f}s (inliers {n})')
            self.pub_cmd.publish(cmd)
            if lost >= self.lost_timeout:
                self.get_logger().warn('Chemin perdu : arrêt', throttle_duration_sec=2.0)
            return None
        self.lost_since = None
        if self.loc.finished(self.s):
            self.finished = True
            self.pub_cmd.publish(cmd)
            self.get_logger().info('Chemin rejoué jusqu\'au bout.')
            return None
        cmd.linear.x = self.v_rep
        # anticipation : vitesse de lacet apprise, mise à l'échelle de la vitesse
        w_ff = kf.w * (self.v_rep / kf.v) if abs(kf.v) > 0.05 else kf.w
        cmd.angular.z = steering_command(dx, self.fx, w_ff, self.k_heading, self.w_max)
        self.pub_cmd.publish(cmd)
        err = heading_error(dx, self.fx)
        self.pub_err.publish(Float32(data=float(err)))
        text.append(f'kf {kf.index}/{len(self.loc.kfs) - 1}  inliers {n}')
        text.append(f'dx {dx:+.1f}px  cap {math.degrees(err):+.1f} deg  w {cmd.angular.z:+.2f}')
        kr, dr = self.loc.features(kf.index)
        return (kr, match(desc, dr))

    def publish_debug(self, bgr):
        m = Image()
        m.header = self.header
        m.height, m.width = bgr.shape[:2]
        m.encoding = 'bgr8'
        m.step = m.width * 3
        m.data = bgr.tobytes()
        self.pub_dbg.publish(m)

    def stop(self):
        if self.mode == 'repeat':
            self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VTRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
