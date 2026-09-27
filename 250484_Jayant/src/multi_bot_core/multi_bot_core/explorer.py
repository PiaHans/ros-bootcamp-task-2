#!/usr/bin/env python3


import math
import threading
from collections import deque
from itertools import permutations

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

FREE_MAX = 40      # cell value < FREE_MAX  -> traversable
OCC_MIN = 65       # cell value >= OCC_MIN  -> obstacle


def latched_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def box_dilate(mask, radius):
    """Square dilation, separable, pure numpy."""
    if radius <= 0:
        return mask
    out = mask
    acc = out.copy()
    for k in range(1, radius + 1):
        acc[k:, :] |= out[:-k, :]
        acc[:-k, :] |= out[k:, :]
    out = acc
    acc = out.copy()
    for k in range(1, radius + 1):
        acc[:, k:] |= out[:, :-k]
        acc[:, :-k] |= out[:, k:]
    return acc


class RobotState:
    def __init__(self, ns):
        self.ns = ns
        self.goal_handle = None
        self.goal_xy = None
        self.busy = False
        self.last_xy = None
        self.last_progress_t = 0.0
        self.going_home = False
        self.goal_id = 0


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')

        self.declare_parameter('robot_namespaces', ['tb3_0', 'tb3_1'])
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame_suffix', 'base_footprint')
        self.declare_parameter('planning_period', 2.0)
        self.declare_parameter('min_frontier_cells', 10)
        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('goal_clearance', 0.30)
        self.declare_parameter('progress_timeout', 40.0)
        self.declare_parameter('progress_distance', 0.30)
        self.declare_parameter('blacklist_radius', 0.6)
        self.declare_parameter('goal_valid_radius', 1.5)
        self.declare_parameter('goal_reached_slack', 1.0)
        self.declare_parameter('gain_radius', 1.5)
        self.declare_parameter('w_distance', 1.0)
        self.declare_parameter('w_gain', 0.35)
        self.declare_parameter('separation_bonus', 2.0)
        self.declare_parameter('separation_reference', 5.0)
        self.declare_parameter('finish_after_empty_cycles', 5)
        self.declare_parameter('return_home', True)
        self.declare_parameter('home_poses', [-4.0, -4.0, 0.0, 4.0, 4.0, 3.1416])

        g = self.get_parameter
        self.namespaces = list(g('robot_namespaces').value)
        self.global_frame = g('global_frame').value
        self.base_suffix = g('base_frame_suffix').value
        self.min_cells = int(g('min_frontier_cells').value)
        self.robot_radius = float(g('robot_radius').value)
        self.goal_clearance = float(g('goal_clearance').value)
        self.progress_timeout = float(g('progress_timeout').value)
        self.progress_distance = float(g('progress_distance').value)
        self.blacklist_radius = float(g('blacklist_radius').value)
        self.goal_valid_radius = float(g('goal_valid_radius').value)
        self.goal_reached_slack = float(g('goal_reached_slack').value)
        self.gain_radius = float(g('gain_radius').value)
        self.w_dist = float(g('w_distance').value)
        self.w_gain = float(g('w_gain').value)
        self.sep_bonus = float(g('separation_bonus').value)
        self.sep_ref = float(g('separation_reference').value)
        self.finish_cycles = int(g('finish_after_empty_cycles').value)
        self.return_home = bool(g('return_home').value)

        home = list(g('home_poses').value)
        self.home = {ns: (home[3 * i], home[3 * i + 1], home[3 * i + 2])
                     for i, ns in enumerate(self.namespaces)} \
            if len(home) == 3 * len(self.namespaces) else {}

        self.map_msg = None
        self.blacklist = []
        self._lock = threading.RLock()
        self._planning = False
        self.empty_cycles = 0
        self.finished = False

        self.cbg = ReentrantCallbackGroup()
        self.robots = {ns: RobotState(ns) for ns in self.namespaces}
        self.nav_clients = {
            ns: ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose',
                             callback_group=self.cbg)
            for ns in self.namespaces
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, g('map_topic').value,
                                 self._on_map, latched_qos())
        self.marker_pub = self.create_publisher(MarkerArray, 'frontiers', 1)
        self.done_pub = self.create_publisher(Bool, 'exploration_done', latched_qos())

        self.create_timer(float(g('planning_period').value), self._plan,
                          callback_group=self.cbg)

        self.get_logger().info(
            f'explorer up for {self.namespaces}, planning on '
            f'"{g("map_topic").value}" in frame "{self.global_frame}"')

    def _on_map(self, msg):
        self.map_msg = msg

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_xy(self, ns):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, f'{ns}/{self.base_suffix}', rclpy.time.Time())
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _frontiers(self):
        m = self.map_msg
        info = m.info
        grid = np.asarray(m.data, dtype=np.int8).reshape(info.height, info.width)
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        unknown = grid < 0
        free = (grid >= 0) & (grid < FREE_MAX)
        occ = grid >= OCC_MIN

        nb = np.zeros_like(unknown)
        nb[1:, :] |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:, :]
        nb[:, 1:] |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        frontier = free & nb

        clearance_cells = int(math.ceil((self.robot_radius + self.goal_clearance) / res))
        frontier &= ~box_dilate(occ.copy(), clearance_cells)

        if not frontier.any():
            return [], grid, info

        clusters = self._cluster(frontier)
        gain_cells = int(math.ceil(self.gain_radius / res))

        out = []
        for comp in clusters:
            if len(comp) < self.min_cells:
                continue
            rows = comp[:, 0]
            cols = comp[:, 1]
            cr, cc = rows.mean(), cols.mean()
            k = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
            r, c = int(rows[k]), int(cols[k])

            r0, r1 = max(0, r - gain_cells), min(info.height, r + gain_cells + 1)
            c0, c1 = max(0, c - gain_cells), min(info.width, c + gain_cells + 1)
            gain = float(np.count_nonzero(unknown[r0:r1, c0:c1])) * res * res

            out.append({
                'xy': (ox + (c + 0.5) * res, oy + (r + 0.5) * res),
                'size': len(comp),
                'gain': gain,
            })
        return out, grid, info

    @staticmethod
    def _cluster(mask):
        h, w = mask.shape
        visited = np.zeros_like(mask)
        comps = []
        for r0, c0 in np.argwhere(mask):
            if visited[r0, c0]:
                continue
            visited[r0, c0] = True
            q = deque([(int(r0), int(c0))])
            comp = []
            while q:
                r, c = q.popleft()
                comp.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not visited[nr, nc]:
                            visited[nr, nc] = True
                            q.append((nr, nc))
            comps.append(np.array(comp))
        return comps

    def _allocate(self, idle, candidates, robot_xy, reserved):
        """Exact min-cost assignment of frontiers to idle robots."""
        usable = []
        for cand in candidates:
            if any(math.dist(cand['xy'], p) < self.blacklist_radius for p in self.blacklist):
                continue
            if any(math.dist(cand['xy'], p) < self.sep_ref * 0.5 for p in reserved):
                continue
            usable.append(cand)
        if not usable:
            return {}

        max_gain = max(c['gain'] for c in usable) or 1.0

        def cost(ns, cand):
            d = math.dist(robot_xy[ns], cand['xy'])
            return self.w_dist * d - self.w_gain * (cand['gain'] / max_gain) * self.sep_ref

        pool = set()
        for ns in idle:
            order = sorted(range(len(usable)), key=lambda j: cost(ns, usable[j]))
            pool.update(order[:6])
        pool = sorted(pool)

        best, best_cost = None, float('inf')
        n = min(len(idle), len(pool))
        for combo in permutations(pool, n):
            total = sum(cost(ns, usable[j]) for ns, j in zip(idle, combo))
            if len(combo) > 1:
                pts = [usable[j]['xy'] for j in combo]
                sep = min(math.dist(a, b)
                          for i, a in enumerate(pts) for b in pts[i + 1:])
                total -= self.sep_bonus * min(sep, self.sep_ref)
            if total < best_cost:
                best_cost, best = total, combo

        return {ns: usable[j] for ns, j in zip(idle, best)} if best else {}

    def _send_goal(self, ns, xy, from_xy, going_home=False):
        client = self.nav_clients[ns]
        if not client.server_is_ready():
            self.get_logger().warn(f'{ns}: navigate_to_pose not ready yet')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        yaw = math.atan2(xy[1] - from_xy[1], xy[0] - from_xy[0])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        st = self.robots[ns]
        st.goal_id += 1
        gid = st.goal_id
        st.busy = True
        st.goal_xy = (float(xy[0]), float(xy[1]))
        st.going_home = going_home
        st.last_xy = from_xy
        st.last_progress_t = self._now()

        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, n=ns, i=gid: self._on_goal_response(n, f, i))
        self.get_logger().info(
            f'{ns} -> ({xy[0]:+.2f}, {xy[1]:+.2f})' + ('  [home]' if going_home else ''))

    def _on_goal_response(self, ns, future, gid):
        st = self.robots[ns]
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'{ns}: goal request failed: {exc}')
            self._release(ns, blacklist=False, gid=gid)
            return
        if not handle.accepted:
            self.get_logger().warn(f'{ns}: goal rejected')
            self._release(ns, blacklist=True, gid=gid)
            return
        with self._lock:
            if gid != st.goal_id:      
                handle.cancel_goal_async()
                return
            st.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, n=ns, i=gid: self._on_result(n, f, i))

    def _on_result(self, ns, future, gid):
        st = self.robots[ns]
        if gid != st.goal_id:          
            return
        try:
            status = future.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._release(ns, blacklist=False, gid=gid)
        else:
            self.get_logger().warn(f'{ns}: goal ended with status {status}')
            self._release(ns, blacklist=not st.going_home, gid=gid)

    def _release(self, ns, blacklist, gid=None):
        with self._lock:
            self._release_locked(ns, blacklist, gid)

    def _release_locked(self, ns, blacklist, gid=None):
        st = self.robots[ns]
        
        if gid is not None and gid != st.goal_id:
            return
        if blacklist and st.goal_xy is not None:
            self.blacklist.append(st.goal_xy)
        st.busy = False
        st.goal_handle = None
        st.goal_xy = None
        st.going_home = False
        st.goal_id += 1

    def _cancel(self, ns, blacklist):
        st = self.robots[ns]
        if st.goal_handle is not None:
            st.goal_handle.cancel_goal_async()
        self._release(ns, blacklist)

    def _plan(self):
        if self.map_msg is None or self.finished:
            return
        if self._planning:            # skip overlapping cycles
            return
        self._planning = True
        try:
            with self._lock:
                self._plan_locked()
        finally:
            self._planning = False

    def _plan_locked(self):
        robot_xy = {ns: self._robot_xy(ns) for ns in self.namespaces}
        if any(v is None for v in robot_xy.values()):
            self.get_logger().warn('waiting for TF of all robots...', once=True)
            return

        now = self._now()

        for ns, st in self.robots.items():
            if not st.busy:
                continue
            if st.last_xy is None or math.dist(robot_xy[ns], st.last_xy) > self.progress_distance:
                st.last_xy = robot_xy[ns]
                st.last_progress_t = now
            elif now - st.last_progress_t > self.progress_timeout:
                self.get_logger().warn(f'{ns}: no progress, re-planning')
                self._cancel(ns, blacklist=True)

        candidates, _, _ = self._frontiers()
        self._publish_markers(candidates)

        for ns, st in self.robots.items():
            if st.busy and not st.going_home and st.goal_xy is not None:
                # nearly there: let Nav2 finish rather than thrashing the goal
                if math.dist(robot_xy[ns], st.goal_xy) < self.goal_reached_slack:
                    continue
                if not any(math.dist(st.goal_xy, c['xy']) < self.goal_valid_radius
                           for c in candidates):
                    self._cancel(ns, blacklist=False)

        if not candidates:
            self.empty_cycles += 1
            if self.empty_cycles >= self.finish_cycles and \
                    not any(r.busy for r in self.robots.values()):
                self._finish(robot_xy)
            return
        self.empty_cycles = 0

        idle = [ns for ns, st in self.robots.items() if not st.busy]
        if not idle:
            return
        reserved = [st.goal_xy for st in self.robots.values()
                    if st.busy and st.goal_xy is not None]

        for ns, cand in self._allocate(idle, candidates, robot_xy, reserved).items():
            self._send_goal(ns, cand['xy'], robot_xy[ns])

    def _finish(self, robot_xy):
        self.finished = True
        self.get_logger().info(
            '=== EXPLORATION COMPLETE - no reachable frontiers left ===')
        self.get_logger().info(
            'save the merged map with:  ros2 run nav2_map_server map_saver_cli '
            '-t /map -f ~/merged_map --ros-args -p use_sim_time:=true')
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)

        if self.return_home and self.home:
            for ns in self.namespaces:
                hx, hy, _ = self.home[ns]
                self._send_goal(ns, (hx, hy), robot_xy[ns], going_home=True)

    def _publish_markers(self, candidates):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'frontiers'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 0.9
        m.pose.orientation.w = 1.0
        for c in candidates:
            m.points.append(Point(x=float(c['xy'][0]), y=float(c['xy'][1]), z=0.1))
        arr.markers.append(m)

        b = Marker()
        b.header = m.header
        b.ns = 'blacklist'
        b.id = 1
        b.type = Marker.SPHERE_LIST
        b.action = Marker.ADD
        b.scale.x = b.scale.y = b.scale.z = 0.25
        b.color.r, b.color.g, b.color.b, b.color.a = 0.9, 0.1, 0.1, 0.7
        b.pose.orientation.w = 1.0
        for p in self.blacklist:
            b.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.1))
        arr.markers.append(b)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
