import sys
from pyrobosim.core.world import World
from pyrobosim.gui.main import start_gui
from pyrobosim.core.robot import Robot
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.utils.pose import Pose

# Programmatically create a world (official demo style)
world = World()

# Add rooms
world.add_room(
	name="kitchen",
	pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
	footprint=[(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)],
	color="red",
	nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
)
world.add_room(
	name="bedroom",
	pose=Pose(x=2.625, y=3.25, z=0.0, yaw=0.0),
	footprint=[(-0.875, -0.75), (0.875, -0.75), (0.875, 0.75), (-0.875, 0.75)],
	color="#009900",
)
world.add_room(
	name="bathroom",
	footprint=[(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)],
	color=[0.0, 0.0, 0.6],
)

# Add hallways
world.add_hallway(room_start="kitchen", room_end="bathroom", width=0.7, color="#666666")
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

# Add robot with lidar and path planner
lidar = Lidar2D(
	update_rate_s=0.1,
	angle_units="degrees",
	min_angle=-120.0,
	max_angle=120.0,
	angular_resolution=5.0,
	max_range_m=2.0,
)
robot = Robot(
	name="robot0",
	radius=0.1,
	path_executor=ConstantVelocityExecutor(linear_velocity=1.0, dt=0.1, max_angular_velocity=4.0),
	sensors={"lidar": lidar},
	color="#CC00CC",
)
world.add_robot(robot, loc="kitchen")
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
robot.set_path_planner(rrt_planner)

# Launch GUI
start_gui(world)
