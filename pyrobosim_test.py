
from pyrobosim.core.world import World
from pyrobosim.core.robot import Robot
from pyrobosim.utils.pose import Pose
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.gui.main import start_gui
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
import subprocess
import sys
import time


# Create world just like demo.py
world = World()

# Add metadata for locations and objects from pyrobosim data folder (like demo.py)
import os
from pyrobosim.utils.general import get_data_folder
data_folder = get_data_folder()
world.add_metadata(
    locations=[
        os.path.join(data_folder, "example_location_data.yaml")
    ],
    objects=[
        os.path.join(data_folder, "example_object_data.yaml")
    ]
)

# Add rooms
r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
world.add_room(
    name="kitchen",
    pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
    footprint=r1coords,
    color="red",
    nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
)
r2coords = [(-0.875, -0.75), (0.875, -0.75), (0.875, 0.75), (-0.875, 0.75)]
world.add_room(
    name="bedroom",
    pose=Pose(x=2.625, y=3.25, z=0.0, yaw=0.0),
    footprint=r2coords,
    color="#009900",
)
r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
world.add_room(
    name="bathroom",
    footprint=r3coords,
    color=[0.0, 0.0, 0.6],
)

# Add hallways
world.add_hallway(
    room_start="kitchen", room_end="bathroom", width=0.7, color="#666666"
)
world.add_hallway(
    room_start="bathroom",
    room_end="bedroom",
    width=0.5,
    conn_method="angle",
    conn_angle=0,
    offset=0.8,
    color="dimgray",
)
world.add_hallway(
    room_start="kitchen",
    room_end="bedroom",
    width=0.6,
    conn_method="points",
    conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
)

# Add locations
table = world.add_location(
    category="table",
    parent="kitchen",
    pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-90.0, angle_units="degrees"),
)
desk_pose = world.get_pose_relative_to(
    Pose(x=0.525, y=0.4, z=0.0, yaw=0.0), "bedroom"
)
desk = world.add_location(category="desk", parent="bedroom", pose=desk_pose)
counter = world.add_location(
    category="counter",
    parent="bathroom",
    pose=Pose(x=-2.45, y=2.5, z=0.0, yaw=0.0),
)

# Add objects
banana_pose = world.get_pose_relative_to(
    Pose(x=0.15, y=0.0, z=0.0, q=[0.9238811, 0.0, 0.0, -0.3826797]), table
)
world.add_object(category="banana", parent=table, pose=banana_pose)
apple_pose = world.get_pose_relative_to(
    Pose(x=0.05, y=-0.15, z=0.0, q=[1.0, 0.0, 0.0, 0.0]), desk
)
world.add_object(category="apple", parent=desk, pose=apple_pose)
world.add_object(category="apple", parent=table)
world.add_object(category="apple", parent=table)
world.add_object(category="water", parent=counter)
world.add_object(category="banana", parent=counter)
world.add_object(category="water", parent=desk)

# Add robot
grasp_props = ParallelGraspProperties(
    max_width=0.175,
    depth=0.1,
    height=0.04,
    width_clearance=0.01,
    depth_clearance=0.01,
)
avoid_left = Lidar2D(
    update_rate_s=0.1,
    angle_units="degrees",
    min_angle=30.0,
    max_angle=50.0,
    angular_resolution=2.0,
    max_range_m=0.5,
)
avoid_right = Lidar2D(
    update_rate_s=0.1,
    angle_units="degrees",
    min_angle=130.0,
    max_angle=150.0,
    angular_resolution=2.0,
    max_range_m=0.5,
)
# Mimic ultrasonic sensor on a servo (centered at 90°, can sweep 0–180°)
# To simulate servo movement, you would update min_angle and max_angle dynamically in code.
avoid_front = Lidar2D(
    update_rate_s=0.1,
    angle_units="degrees",
    min_angle=0.0,  # narrow beam, center at 90°
    max_angle=180.0,
    angular_resolution=1.0,
    max_range_m=0.3,  # typical ultrasonic range
)
robot0 = Robot(
    name="robot0",
    radius=0.1,
    path_executor=ConstantVelocityExecutor(
        linear_velocity=1.0,
        dt=0.1,
        max_angular_velocity=4.0,
        validate_during_execution=True,
    ),
    sensors={
        "avoid_left": avoid_left,
        "avoid_right": avoid_right,
        "avoid_front": avoid_front,
    },
    grasp_generator=GraspGenerator(grasp_props),
    partial_obs_objects=False,
    color="#CC00CC",
)
world.add_robot(robot0, loc="kitchen")
planner_config_rrt = {
    "bidirectional": True,
    "rrt_connect": False,
    "rrt_star": True,
    "collision_check_step_dist": 0.025,
    "max_connection_dist": 0.5,
    "rewire_radius": 1.5,
    "compress_path": False,
}
rrt_planner = RRTPlanner(**planner_config_rrt)
robot0.set_path_planner(rrt_planner)


# Start GUI
start_gui(world)


# Add obstacles (boxes) to the world
obstacle_poses = [
    Pose(x=1.0, y=0.0, z=0.0),
    Pose(x=2.0, y=1.0, z=0.0),
    Pose(x=0.0, y=2.0, z=0.0),
    Pose(x=1.5, y=1.5, z=0.0),
]
for i, pose in enumerate(obstacle_poses):
    world.add_object(category="box", pose=pose, name=f"obstacle_{i+1}")

# Run auto_avoid.py logic in a subprocess while robot navigates
import subprocess
import sys
import threading
import time

def navigate_robot_with_autoavoid(robot, locations):
    proc = subprocess.Popen([
        sys.executable, "auto_avoid.py", "--threshold-cm", "30", "--speed", "45"
    ])
    try:
        while True:
            for loc in locations:
                print(f"[robot0] Navigating to {loc.name} using auto_avoid")
                robot.navigate_to(loc)
                while robot.is_navigating:
                    time.sleep(0.1)
            # Loop forever
    finally:
        proc.terminate()

all_locations = [table, desk, counter]
nav_thread = threading.Thread(target=navigate_robot_with_autoavoid, args=(robot0, all_locations), daemon=True)
nav_thread.start()
