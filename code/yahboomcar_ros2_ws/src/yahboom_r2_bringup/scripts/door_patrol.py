#!/usr/bin/env python3
"""Scripted R2 mission: right turn ~90deg, straight 2 m, left turn ~90deg (to the door),
then wander forward within a 2 m radius (steering back toward the centre at the edge)
until patrol_time or low battery, then drive forward along the outbound route back to the start.
Reversing is limited to max_reverse_time per manoeuvre (only when the front is blocked).
Wheel slip: over a slip_window, if wheel odometry moved >= slip_min_dist but the
Cartographer pose (map->base_link) moved < slip_ratio of it, the car stops and the run ends.
Odometry: /odom (EKF).
Battery: /dev/shm/r2_battery_v written by yahboom_r2_hardware.
"""
import math, time, os, csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
import tf2_ros


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class DoorPatrol(Node):
    def __init__(self):
        super().__init__('door_patrol')
        p = self.declare_parameter
        self.v = p('speed', 0.10).value
        self.w_turn = p('turn_yaw_rate', 0.2).value
        self.fwd_dist = p('forward_dist', 2.0).value
        self.turn1 = math.radians(p('turn1_deg', -90.0).value)
        self.turn2 = math.radians(p('turn2_deg', 90.0).value)
        self.leg = p('patrol_leg', 1.5).value
        self.radius = p('patrol_radius', 2.0).value
        self.patrol_time = p('patrol_time', 300.0).value
        self.batt_low = p('battery_low_v', 10.8).value
        self.stop_d = p('stop_dist', 0.30).value
        self.self_d = p('self_mask_dist', 0.18).value
        self.max_rev = p('max_reverse_time', 3.0).value
        self.slip_win = p('slip_window', 2.0).value
        self.slip_min = p('slip_min_dist', 0.12).value
        self.slip_ratio = p('slip_ratio', 0.5).value
        self.wheel_topic = p('wheel_odom_topic', '/ackermann_steering_controller/odometry').value
        self.out_dir = p('out_dir', os.path.expanduser('~/r2_runs/door')).value
        os.makedirs(self.out_dir, exist_ok=True)
        self.pose = None
        self.wheel = None
        self.slip_hist = []
        self.rev_until = 0.0
        self.rev_sign = 1.0
        self.blocked_since = None
        self.front = None
        self.left = self.right = 1.0
        self.map = None
        self.batt = None
        self.batt_low_since = None
        self.low_batt = False
        self.state = 'WAIT'
        self.ref = None
        self.outbound = []
        self.ret_path = []
        self.leg_n = 0
        self.patrol_t0 = None
        self.path = []
        self.t0 = time.time()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Odometry, self.wheel_topic, self.on_wheel, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), mq)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_timer(0.1, self.step)
        self.create_timer(0.2, self.record)
        self.create_timer(1.0, self.read_batt)
        self.log = open(os.path.join(self.out_dir, 'log.csv'), 'w', newline='')
        self.logw = csv.writer(self.log)
        self.logw.writerow(['t', 'state', 'x', 'y', 'yaw', 'front', 'batt', 'vx', 'wz'])

    def on_odom(self, m):
        pp = m.pose.pose
        self.pose = (pp.position.x, pp.position.y, yaw_of(pp.orientation))

    def on_wheel(self, m):
        pp = m.pose.pose.position
        self.wheel = (pp.x, pp.y)

    def check_slip(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return False
        if self.wheel is None:
            return False
        now = time.time()
        self.slip_hist.append((now, self.wheel, (t.transform.translation.x, t.transform.translation.y)))
        while self.slip_hist and now - self.slip_hist[0][0] > self.slip_win:
            self.slip_hist.pop(0)
        if len(self.slip_hist) < 5 or now - self.slip_hist[0][0] < self.slip_win * 0.8:
            return False
        _, w0, m0 = self.slip_hist[0]
        _, w1, m1 = self.slip_hist[-1]
        dw = math.hypot(w1[0] - w0[0], w1[1] - w0[1])
        dm = math.hypot(m1[0] - m0[0], m1[1] - m0[1])
        if dw >= self.slip_min and dm < self.slip_ratio * dw:
            self.get_logger().error(f'WHEEL SLIP: wheel {dw:.2f} m, actual {dm:.2f} m -> stop')
            return True
        return False

    def on_scan(self, m):
        fr, lf, rt = [], [], []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < self.self_d:
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment)
            a = (a + 180.0) % 360.0 - 180.0
            if abs(a) <= 20:
                fr.append(r)
            if 15 <= a <= 75:
                lf.append(min(r, 2.5))
            if -75 <= a <= -15:
                rt.append(min(r, 2.5))
        self.front = min(fr) if fr else 3.0
        self.left = float(np.percentile(lf, 30)) if lf else 0.0
        self.right = float(np.percentile(rt, 30)) if rt else 0.0

    def read_batt(self):
        try:
            self.batt = float(open('/dev/shm/r2_battery_v').read())
        except Exception:
            return
        if self.batt < self.batt_low:
            self.batt_low_since = self.batt_low_since or time.time()
            if time.time() - self.batt_low_since > 15 and not self.low_batt:
                self.low_batt = True
                self.get_logger().warn(f'battery low {self.batt:.1f} V -> returning')
        else:
            self.batt_low_since = None

    def record(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.path.append((t.transform.translation.x, t.transform.translation.y))
        except Exception:
            pass

    def set_state(self, s):
        self.get_logger().info(f'{self.state} -> {s}')
        self.state = s
        self.ref = self.pose

    def dist_from_ref(self):
        return math.hypot(self.pose[0] - self.ref[0], self.pose[1] - self.ref[1])

    def step(self):
        cmd = Twist()
        if self.pose is None or self.front is None:
            self.pub.publish(cmd)
            return
        if self.state not in ('WAIT', 'DONE', 'SLIP') and self.check_slip():
            self.set_state('SLIP')
        if self.state == 'SLIP':
            self.pub.publish(Twist())
            raise SystemExit
        x, y, yaw = self.pose
        blocked = self.front < self.stop_d
        if self.state in ('TURN1', 'FWD', 'TURN2') and (not self.outbound or
                math.hypot(x - self.outbound[-1][0], y - self.outbound[-1][1]) > 0.05):
            self.outbound.append((x, y))
        s = self.state
        if s == 'WAIT':
            self.set_state('TURN1')
            self.outbound.append((x, y))
        elif s in ('TURN1', 'TURN2'):
            target = self.turn1 if s == 'TURN1' else self.turn2
            done = abs(wrap(yaw - self.ref[2])) >= abs(target) - math.radians(5)
            if done:
                self.set_state('FWD' if s == 'TURN1' else 'PATROL')
                if s == 'TURN2':
                    self.center = (x, y)
                    self.patrol_t0 = time.time()
            elif not blocked:
                cmd.linear.x = self.v
                cmd.angular.z = math.copysign(self.w_turn, target)
        elif s == 'FWD':
            if self.dist_from_ref() >= self.fwd_dist:
                self.set_state('TURN2')
            elif not blocked:
                cmd.linear.x = self.v
                cmd.angular.z = 1.5 * wrap(self.ref[2] - yaw)  # hold heading
        elif s == 'PATROL':
            end = self.low_batt or time.time() - self.patrol_t0 > self.patrol_time
            if end:
                self.ret_path = list(reversed(self.outbound))
                self.set_state('RETURN')
            else:
                cmd.linear.x, cmd.angular.z = self.wander()
        elif s == 'RETURN':
            while len(self.ret_path) > 1 and math.hypot(x - self.ret_path[0][0], y - self.ret_path[0][1]) < 0.35:
                self.ret_path.pop(0)
            goal = self.ret_path[0]
            if len(self.ret_path) == 1 and math.hypot(x - goal[0], y - goal[1]) < 0.2:
                self.set_state('DONE')
            else:
                cmd.linear.x, cmd.angular.z = self.drive_or_escape(self.pursue(goal))
        elif s == 'DONE':
            self.pub.publish(Twist())
            raise SystemExit
        self.pub.publish(cmd)
        self.logw.writerow([round(time.time() - self.t0, 2), self.state, round(x, 3), round(y, 3),
                            round(yaw, 3), round(self.front, 2), self.batt, round(cmd.linear.x, 3),
                            round(cmd.angular.z, 3)])

    def pursue(self, goal):
        """Forward pure pursuit toward goal; returns desired yaw rate."""
        x, y, yaw = self.pose
        dx, dy = goal[0] - x, goal[1] - y
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        L2 = max(lx * lx + ly * ly, 0.09)
        if lx < 0:  # goal behind: full lock toward it
            return math.copysign(0.4, ly if abs(ly) > 1e-3 else 1.0)
        return max(-0.4, min(0.4, self.v * 2.0 * ly / L2))

    def wander(self):
        x, y, _ = self.pose
        d = math.hypot(x - self.center[0], y - self.center[1])
        if d > self.radius - 0.3:
            w = self.pursue(self.center)
        else:
            w = max(-0.4, min(0.4, 0.4 * (self.left - self.right)))
        return self.drive_or_escape(w)

    def drive_or_escape(self, w):
        """Forward with yaw rate w; if the front is blocked, back up (max max_reverse_time) swinging
        the nose toward the freer side, then wait before reversing again."""
        now = time.time()
        if now < self.rev_until:
            return -self.v, -self.rev_sign * 0.4
        if self.front < self.stop_d:
            if self.blocked_since is None or now - self.blocked_since > self.max_rev + 1.0:
                self.blocked_since = now
                self.rev_sign = 1.0 if self.left >= self.right else -1.0
                self.rev_until = now + self.max_rev
                return -self.v, -self.rev_sign * 0.4
            return 0.0, 0.0
        if self.front < 0.7:
            w = (1.0 if self.left >= self.right else -1.0) * 0.4
        return self.v, w

    def save(self):
        self.log.close()
        with open(os.path.join(self.out_dir, 'path.csv'), 'w') as f:
            for x, y in self.path:
                f.write(f'{x:.3f},{y:.3f}\n')
        if self.map is None:
            return
        import cv2
        m = self.map
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        img = np.full(g.shape, 205, np.uint8)
        img[g == 0] = 255
        img[g > 50] = 0
        img = cv2.cvtColor(np.flipud(img), cv2.COLOR_GRAY2BGR)
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        pts = [(int((x - ox) / res), m.info.height - 1 - int((y - oy) / res)) for x, y in self.path]
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, (0, 0, 255), 1)
        if pts:
            cv2.circle(img, pts[0], 3, (0, 180, 0), -1)
            cv2.circle(img, pts[-1], 3, (255, 0, 0), -1)
        scale = max(1, int(600 / max(img.shape[:2])))
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(os.path.join(self.out_dir, 'map_path.png'), img)


def main():
    rclpy.init()
    n = DoorPatrol()
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    for _ in range(5):
        n.pub.publish(Twist())
        time.sleep(0.05)
    n.save()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
