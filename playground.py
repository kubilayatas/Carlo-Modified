"""
CARLO Çizgi İzleyen Labirent Simülatörü - OYUN ALANI (PLAYGROUND)

Bu dosya, çeşitli path-planning (yol bulma) algoritmalarını ve
kontrolcüleri (PID, Bang-Bang vb.) hızlıca test etmeniz için
tek sayfada toplanmış bir şablondur.
"""

import numpy as np
import time
from world import World
from agents import Painting
from geometry import Point
from line_follower import LineFollowerRobot
from maze_track import MazeTrack
from maze_solver import MazePlanner, PathExecutor
from maze_presets import MAZE_MEDIUM_T, MAZE_EXPERT, MAZE_MineOne, MAZE_SIMPLE_S, MAZE_HARD_DEADENDS

# ==========================================================
# 1. YOL BULMA HESAPLAYICISI (PATH PLANNING ALGORITHM)
# ==========================================================
def custom_path_planner(maze, start_cell, end_cell):
    """
    Kendi yol bulma mantığınızı (A*, Dijkstra vb.) buraya yazabilirsiniz.
    Haritanın matris(grid) hali: maze.grid
    Hangi hücreye nereden gidileceğinin sözlüğü: maze.get_adjacency()
    """
    planner = MazePlanner(maze)
    
    # Hazır algoritmalar arasından seçim yapabilirsiniz:
    path = planner.bfs(start_cell, end_cell)
    # path = planner.flood_fill_path(start_cell, end_cell)
    # path = planner.left_wall_follower(start_cell, end_cell, initial_direction='up')
    #path = planner.right_wall_follower(start_cell, end_cell, initial_direction='right')
    
    return path

# ==========================================================
# 2. ÇİZGİ İZLEME KONTROLCÜLERİ (STEERING CONTROLLERS)
# ==========================================================
class BangBangController:
    """ Basit "Eğer soldaysa Sağa Dön, Sağdaysa Sola Dön" kontrolcüsü. """
    def compute(self, line_error, dt):
        # line_error değeri -1.0 (en sol) ile +1.0 (en sağ) arasında gelir.
        if line_error < -0.1:
            return -1.0  # Tam Sola kır
        elif line_error > 0.1:
            return 1.0   # Tam Sağa kır
        else:
            return 0.0   # Düz git

class CustomPIDController:
    """ Standart PID Algaritması (maze_solver içindekiyle aynı arayüz) """
    def __init__(self, kp=15, ki=0.01, kd=2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_error = error
        return max(-1.0, min(1.0, output)) # Limit kontrolü (-1, +1) aralığı


# *** KULLANILACAK KONTROLCÜYÜ BURADAN SEÇİN ***
#my_controller = BangBangController()
my_controller = CustomPIDController(kp=8, ki=0.02, kd=1)

# ==========================================================
# 3. SİMÜLASYON AYARLARI (HARİTA VE FİZİK)
# ==========================================================
MAZE_GRID = MAZE_EXPERT       # Seçilen Harita: MAZE_MineOne, MAZE_MEDIUM_T vb.
BASE_SPEED = 0.5             # Robotun Düz Yol Hızı (m/s)
CELL_SIZE = 0.2               # Her Bir Labirent Karesinin Boyutu (Metre)
SIM_DT = 0.05                  # Zaman Adımı
ROBOT_SENSOR_COUNT = 8        # IR Sensör Sayısı



# ==========================================================
# ------------------ SİMÜLASYON MOTORU ---------------------
# (Genellikle buranın altını değiştirmenize gerek yoktur)
# ==========================================================
rows, cols = MAZE_GRID.shape
world_width = (cols + 2) * CELL_SIZE
world_height = (rows + 2) * CELL_SIZE

# Dünya
w = World(SIM_DT, width=world_width, height=world_height, ppm=300)
# Beyaz Zemin
w.add(Painting(Point(world_width / 2, world_height / 2), Point(world_width, world_height), 'white'))

# Labirent (Grid'i fiziksel Painting'lere dönüştür)
maze = MazeTrack(MAZE_GRID, cell_size=CELL_SIZE, line_width=0.03, origin_x=CELL_SIZE, origin_y=CELL_SIZE)
maze.add_to_world(w)

start_pos, start_cell = maze.get_start_position()
end_pos, end_cell = maze.get_end_position()

# 1) Rota Oluştur
path = custom_path_planner(maze, start_cell, end_cell)
print(f"Planlanan yol ({len(path)} adım): {path}")

# İlk hareket yönünü (heading) otomatik belirlemek
if len(path) > 1:
    dr = path[1][0] - path[0][0]
    dc = path[1][1] - path[0][1]
    if dr == -1: initial_heading = np.pi / 2       # Yukarı
    elif dr == 1: initial_heading = 3 * np.pi / 2  # Aşağı
    elif dc == -1: initial_heading = np.pi         # Sola
    else: initial_heading = 0.0                    # Sağa
else:
    initial_heading = np.pi / 2

# Robotu dünyaya ekle
robot = LineFollowerRobot(start_pos, heading=initial_heading, sensor_count=ROBOT_SENSOR_COUNT)
robot.velocity = Point(0, 0)
w.add(robot)
robot.attach_to_world(w)

# Sürücü
executor = PathExecutor(robot, maze, my_controller, path, base_speed=BASE_SPEED, junction_threshold=0.05)

# GUI ve Döngü
w.render()
max_ticks = 5000

for k in range(max_ticks):
    executor.step(SIM_DT)
    w.tick()
    w.render()
    time.sleep(SIM_DT / 4)

    # 10 adımda bir konsola sensör durumunu yazdır
    if k % 10 == 0:
        print(f"Adım {k} | Durum: {robot.sensor_debug_str} | Hız: {robot.speed:.2f} m/s")

    if executor.finished:
        print(f"Hedefe Ulaşıldı! Görev Adımı: {k}")
        break

    if w.collision_exists(robot):
        print("Çarpışma Algılandı, Simülasyon Durduruluyor!")
        break

time.sleep(2)
w.close()
