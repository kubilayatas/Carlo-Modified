import numpy as np
import time
from world import World
from agents import Painting
from geometry import Point
from line_follower import LineFollowerRobot
from maze_track import MazeTrack
from maze_solver import MazePlanner, PathExecutor
from pid_controller import PIDController
from maze_presets import MAZE_MEDIUM_T, MAZE_EXPERT, MAZE_MineOne

# ---- Konfigürasyon ----
dt = 0.1
cell_size = 0.2
maze_grid = MAZE_EXPERT
rows, cols = maze_grid.shape
world_width = (cols + 2) * cell_size     # +2 kenar boşluk
world_height = (rows + 2) * cell_size

# ---- Dünya oluştur ----
w = World(dt, width=world_width, height=world_height, ppm=300)

# ---- Zemin (beyaz) ----
w.add(Painting(Point(world_width / 2, world_height / 2),
               Point(world_width, world_height), 'white'))

# ---- Labirent çiz ----
maze = MazeTrack(maze_grid, cell_size=cell_size, line_width=0.02,
                 origin_x=cell_size, origin_y=cell_size)
maze.add_to_world(w)

# ---- Pozisyonları Al ----
start_pos, start_cell = maze.get_start_position()
end_pos, end_cell = maze.get_end_position()

# ---- Yol planla ----
planner = MazePlanner(maze)
path = planner.bfs(start_cell, end_cell)
print(f"Planlanan yol ({len(path)} adım): {path}")

# İlk hareket yönünü (heading) otomatik belirle
if len(path) > 1:
    dr = path[1][0] - path[0][0]
    dc = path[1][1] - path[0][1]
    if dr == -1: initial_heading = np.pi / 2       # Yukarı
    elif dr == 1: initial_heading = 3 * np.pi / 2  # Aşağı
    elif dc == -1: initial_heading = np.pi         # Sola
    else: initial_heading = 0.0                    # Sağa
else:
    initial_heading = np.pi / 2

# ---- Robot yerleştir ----
robot = LineFollowerRobot(start_pos, heading=initial_heading, sensor_count=8)
robot.velocity = Point(0, 0)
w.add(robot)
robot.attach_to_world(w)

# Alternatifler:
# path = planner.flood_fill_path(start_cell, end_cell)
# path = planner.left_wall_follower(start_cell, end_cell, initial_direction='up')

# ---- PID + Executor ----
pid = PIDController(kp=10.0, ki=0.01, kd=1.5, output_min=-5.0, output_max=5.0)
executor = PathExecutor(robot, maze, pid, path, base_speed=0.4, junction_threshold=0.15,
                        junction_pause_ticks=15)

# ---- Simülasyon döngüsü ----
w.render()
max_ticks = 2000

for k in range(max_ticks):
    executor.step(dt)
    w.tick()
    w.render()
    time.sleep(dt / 4)

    if executor.finished:
        print(f"Labirent çözüldü! Adım: {k}, İlerleme: {executor.progress:.1f}%")
        break

    if k % 50 == 0:
        print(f"Tick {k}: Sensörler={robot.sensor_debug_str}, "
              f"İlerleme={executor.progress:.1f}%, Hız={robot.speed:.1f} m/s")

    if w.collision_exists(robot):
        print("Robot çarpıştı!")
        break

time.sleep(2)
w.close()
