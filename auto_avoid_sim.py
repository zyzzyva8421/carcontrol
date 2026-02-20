#!/usr/bin/env python3
"""2D simulator for auto obstacle avoidance algorithm.

Run: python3 auto_avoid_sim.py
"""
import math
import time
import argparse
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Rect:
    x: float
    y: float
    w: float
    h: float

    def contains_point(self, px: float, py: float) -> bool:
        return self.x - self.w/2 <= px <= self.x + self.w/2 and self.y - self.h/2 <= py <= self.y + self.h/2

    def edges(self) -> List[Tuple[Tuple[float,float], Tuple[float,float]]]:
        x1 = self.x - self.w/2
        x2 = self.x + self.w/2
        y1 = self.y - self.h/2
        y2 = self.y + self.h/2
        return [((x1,y1),(x2,y1)), ((x2,y1),(x2,y2)), ((x2,y2),(x1,y2)), ((x1,y2),(x1,y1))]


def ray_intersect_segment(rx, ry, dx, dy, x1, y1, x2, y2) -> Optional[float]:
    # ray: (rx,ry) + t*(dx,dy), t>=0
    # segment: (x1,y1)-(x2,y2)
    vx = x2 - x1
    vy = y2 - y1
    denom = dx * vy - dy * vx
    if abs(denom) < 1e-9:
        return None
    t = ((x1 - rx) * vy - (y1 - ry) * vx) / denom
    u = ((x1 - rx) * dy - (y1 - ry) * dx) / denom
    if t >= 0 and 0 <= u <= 1:
        return t
    return None


def raycast(rx, ry, angle_rad, max_dist, obstacles: List[Rect]) -> Optional[float]:
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)
    closest = None
    for obs in obstacles:
        for (x1,y1),(x2,y2) in obs.edges():
            t = ray_intersect_segment(rx, ry, dx, dy, x1, y1, x2, y2)
            if t is not None and t <= max_dist:
                if closest is None or t < closest:
                    closest = t
    return closest


class RobotSim:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0

    def step(self, dt):
        self.theta += self.omega * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt


def clamp(v, a, b):
    return max(a, min(b, v))


def sim_main(args):
    # world setup
    obstacles = [
        Rect(0.8, 0.0, 0.2, 0.2),
        Rect(-0.6, -0.2, 0.3, 0.3),
    ]

    robot = RobotSim(x=0.0, y=0.0, theta=0.0)

    max_us_m = 2.0
    threshold_m = args.threshold_cm / 100.0
    prefer_left = True

    t = 0.0
    dt = args.loop_seconds
    steps = int(args.total_time / dt)
    traj = []

    for step in range(steps):
        # sensors
        us = raycast(robot.x, robot.y, robot.theta, max_us_m, obstacles)
        us_m = us if us is not None else max_us_m
        # IR sensors: check small side points
        side_offset = 0.09
        left_x = robot.x + math.cos(robot.theta + math.pi/2) * side_offset
        left_y = robot.y + math.sin(robot.theta + math.pi/2) * side_offset
        right_x = robot.x + math.cos(robot.theta - math.pi/2) * side_offset
        right_y = robot.y + math.sin(robot.theta - math.pi/2) * side_offset
        ir_left = any(obs.contains_point(left_x, left_y) for obs in obstacles)
        ir_right = any(obs.contains_point(right_x, right_y) for obs in obstacles)

        too_close = us_m < threshold_m

        print(f"t={t:.2f} IR L={ir_left} R={ir_right} US={us_m:.3f}")

        if too_close or ir_left or ir_right:
            # simple brake
            # if both IR blocked -> back up
            if ir_left and ir_right:
                # move backward for back_seconds
                dur = args.back_seconds
                robot.v = -args.speed_m
                for _ in range(int(dur / dt)):
                    robot.step(dt)
                    t += dt
                robot.v = 0.0
                continue

            # sweep
            delta = math.radians(40)
            left_d = raycast(robot.x, robot.y, robot.theta + delta, max_us_m, obstacles)
            right_d = raycast(robot.x, robot.y, robot.theta - delta, max_us_m, obstacles)
            l = left_d if left_d is not None else max_us_m
            r = right_d if right_d is not None else max_us_m

            if ir_left:
                choice = 'right'
            elif ir_right:
                choice = 'left'
            else:
                if l < threshold_m and r < threshold_m:
                    choice = 'both'
                else:
                    choice = 'left' if l > r else 'right'

            print(f"  sweep L={l:.3f} R={r:.3f} choice={choice}")

            if choice == 'both':
                # back then turn
                robot.v = -args.speed_m
                for _ in range(int(args.back_seconds / dt)):
                    robot.step(dt)
                    t += dt
                robot.v = 0.0
                # turn in place
                turn_steps = int(args.turn_seconds / dt)
                if prefer_left:
                    robot.omega = args.turn_rate
                else:
                    robot.omega = -args.turn_rate
                for _ in range(turn_steps):
                    robot.step(dt)
                    t += dt
                robot.omega = 0.0
            elif choice == 'left':
                robot.omega = args.turn_rate
                for _ in range(int(args.turn_seconds / dt)):
                    robot.step(dt)
                    t += dt
                robot.omega = 0.0
            else:
                robot.omega = -args.turn_rate
                for _ in range(int(args.turn_seconds / dt)):
                    robot.step(dt)
                    t += dt
                robot.omega = 0.0

            prefer_left = not prefer_left
        else:
            # move forward
            robot.v = args.speed_m
            robot.step(dt)
            t += dt

        traj.append((robot.x, robot.y))

    print(f"Final pose: x={robot.x:.2f} y={robot.y:.2f} theta={robot.theta:.2f}")

    if args.plot:
        try:
            import matplotlib.pyplot as plt

            xs = [p[0] for p in traj]
            ys = [p[1] for p in traj]
            plt.figure(figsize=(6,6))
            # plot obstacles
            for obs in obstacles:
                rect = plt.Rectangle((obs.x-obs.w/2, obs.y-obs.h/2), obs.w, obs.h, color='red', alpha=0.5)
                plt.gca().add_patch(rect)
            plt.plot(xs, ys, '-o', markersize=2)
            plt.scatter([xs[0]], [ys[0]], c='green', label='start')
            plt.scatter([xs[-1]], [ys[-1]], c='blue', label='end')
            plt.axis('equal')
            plt.title('Auto-avoid sim trajectory')
            plt.legend()
            plt.grid(True)
            plt.savefig(args.plot_file)
            print(f"Saved plot to {args.plot_file}")
        except Exception as exc:
            print(f"Plot failed: {exc}")


def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument('--threshold-cm', type=float, default=30.0)
    p.add_argument('--speed-cm-s', type=float, default=20.0)
    p.add_argument('--loop-seconds', type=float, default=0.05)
    p.add_argument('--back-seconds', type=float, default=0.2)
    p.add_argument('--turn-seconds', type=float, default=0.3)
    p.add_argument('--total-time', type=float, default=10.0)
    p.add_argument('--plot', action='store_true', help='Show/save matplotlib plot of trajectory')
    p.add_argument('--plot-file', default='webots/logs/sim_plot.png', help='Path to save plot')
    return p


if __name__ == '__main__':
    args = build_parser().parse_args()
    # convert speeds
    args.speed_m = args.speed_cm_s / 100.0
    args.turn_rate = math.radians(90)  # rad/s for turning
    sim_main(args)
