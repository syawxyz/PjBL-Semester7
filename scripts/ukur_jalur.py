#!/usr/bin/env python3
"""Minta jalur global ke planner_server Nav2, lalu ukur panjang dan total perubahan arahnya.

Jalankan saat simulasi tb3_simulation_launch.py sudah berjalan:
    python3 scripts/ukur_jalur.py <nama> <output.csv> [goal_x goal_y]
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

START = (-2.0, -0.5)  # posisi spawn TurtleBot3 di tb3_simulation_launch.py


def main():
    name, out = sys.argv[1], sys.argv[2]
    goal = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (1.5, 0.5)

    rclpy.init()
    node = Node('ukur_jalur')

    # Beri initial pose sampai AMCL membalas lewat /amcl_pose.
    got = []
    node.create_subscription(
        PoseWithCovarianceStamped, 'amcl_pose', got.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    for _ in range(60):
        if got:
            break
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = START
        msg.pose.pose.orientation.w = 1.0
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=1.0)
    if not got:
        sys.exit('AMCL tidak membalas initial pose')

    # Tunggu costmap global terisi peta; tanpa ini jalur bisa menembus halangan.
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    client = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    if not client.wait_for_server(timeout_sec=30):
        sys.exit('Action compute_path_to_pose tidak tersedia')
    req = ComputePathToPose.Goal()
    req.goal.header.frame_id = 'map'
    req.goal.pose.position.x, req.goal.pose.position.y = goal
    req.goal.pose.orientation.w = 1.0
    req.use_start = False  # mulai dari posisi robot saat ini
    req.planner_id = 'GridBased'

    for _ in range(5):
        t0 = time.monotonic()
        fut = client.send_goal_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
        handle = fut.result()
        if handle is None or not handle.accepted:
            time.sleep(2)
            continue
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=30)
        wall_ms = (time.monotonic() - t0) * 1000
        result = res_fut.result()
        if result.status == 4 and result.result.path.poses:  # 4 = SUCCEEDED
            break
        time.sleep(2)
    else:
        sys.exit('Planner gagal membuat jalur')

    pts = [(p.pose.position.x, p.pose.position.y) for p in result.result.path.poses]
    length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(pts, pts[1:]) if math.dist(a, b) > 1e-6]
    turn = sum(abs(math.degrees(math.atan2(math.sin(b - a), math.cos(b - a))))
               for a, b in zip(headings, headings[1:]))

    with open(out, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['x', 'y'])
        writer.writerows(pts)
    print(f'{name}: panjang = {length:.2f} m, total perubahan arah = {turn:.0f} deg, '
          f'waktu (sisi client) = {wall_ms:.0f} ms, titik = {len(pts)}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
