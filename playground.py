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
from maze_solver import MazePlanner, PathExecutor, DynamicExplorer
from maze_presets import MAZE_MEDIUM_T, MAZE_EXPERT, MAZE_MineOne, MAZE_SIMPLE_S, MAZE_HARD_DEADENDS

# ==========================================================
# 1. YOL BULMA HESAPLAYICISI (PATH PLANNING ALGORITHM)
# ==========================================================
# (ESKİ) def custom_path_planner(maze, start_cell, end_cell):
# ... Bu kısım kullanıcının talebi üzerine Faz 1 (dinamik keşif) mantığıyla değiştirildiği için 
# statik planlayıcı iptal edilmiştir. Robot yolu önceden bilmeyecektir.

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
        return max(-1.2, min(1.2, output)) # Limit kontrolü (-1.2, +1.2) aralığı (Çok keskin dönüş)


# *** KULLANILACAK KONTROLCÜYÜ BURADAN SEÇİN ***
my_controller = BangBangController()
# Not: kp=2115 çok yüksek bir değerdir ve sistemi Bang-Bang kontrolcüye çevirir (-1 ile +1 arası salınım).
# Yumuşak bir izleme için değerleri düşürdük:
#my_controller = CustomPIDController(kp=2.0, ki=0.001, kd=0.2)

# ==========================================================
# 3. SİMÜLASYON AYARLARI (HARİTA VE FİZİK)
# ==========================================================
MAZE_GRID = MAZE_EXPERT       # Seçilen Harita: MAZE_MineOne, MAZE_MEDIUM_T vb.
BASE_SPEED = 0.4              # Robotun Düz Yol Hızı (m/s)
CELL_SIZE = 0.2               # Her Bir Labirent Karesinin Boyutu (Metre)
SIM_DT = 0.05                 # Zaman Adımı

# Robot ve Sensör Ayarları
ROBOT_SENSOR_COUNT = 8        # IR Sensör Sayısı
SENSOR_SPREAD = 0.056         # Sensör dizilimi genişliği (Metre, çok açıksa çizgiyi görmeyebilir)
SENSOR_OFFSET = 0.08         # Sensörlerin robot merkezinden ne kadar önde olduğu (Metre)

# Çizgi ve Navigasyon Ayarları
LINE_WIDTH = 0.03             # Yerdeki çizginin kalınlığı (Metre)
JUNCTION_THRESHOLD = 0.05     # Kavşak merkezine varış kabul edilme hassasiyeti (Metre)
LOST_LINE_TIMEOUT = 500        # Çizgi kaybedildiğinde kaç adım boyunca hedefe yönelmeye çalışacak



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
maze = MazeTrack(MAZE_GRID, cell_size=CELL_SIZE, line_width=LINE_WIDTH, origin_x=CELL_SIZE, origin_y=CELL_SIZE)
maze.add_to_world(w)

start_pos, start_cell = maze.get_start_position()
end_pos, end_cell = maze.get_end_position()

# 1) DİNAMİK KEŞİF BAŞLATICISI (FAZ 1 BAŞLANGICI)

# Başlangıç konumundan çıkan ilk açık yolu otomatik algıla:
start_topology = maze.get_topology_at(*start_cell)
initial_heading_str = 'up'
initial_heading = np.pi / 2

if start_topology:
    for d, angle in [('up', np.pi/2), ('down', 3*np.pi/2), ('left', np.pi), ('right', 0.0)]:
        if start_topology.get(d, False):
            initial_heading_str = d
            initial_heading = angle
            break

explorer = DynamicExplorer(maze_track=maze, start_cell=start_cell, initial_heading_str=initial_heading_str)

# Robotu dünyaya ekle
robot = LineFollowerRobot(start_pos, heading=initial_heading, sensor_count=ROBOT_SENSOR_COUNT,
                          sensor_spread=SENSOR_SPREAD, sensor_forward_offset=SENSOR_OFFSET)
robot.velocity = Point(0, 0)
w.add(robot)
robot.attach_to_world(w)

# Sürücü (Faz 1 için path önceden verilmez)
executor = PathExecutor(robot, maze, my_controller, path=None, base_speed=BASE_SPEED, 
                        junction_threshold=JUNCTION_THRESHOLD, lost_line_timeout=LOST_LINE_TIMEOUT)

# İlk dinamik hücre hedefi robotun kendi hücresinin topolojisinden seçilir:
next_cell = explorer.discover_and_decide_next(start_cell)
if next_cell:
    executor.set_dynamic_target(next_cell)

# GUI ve Döngü
w.render()
max_ticks = 5000
phase_1_done = False

print(">>> FAZ 1 (KEŞİF) BAŞLADI <<<")
for k in range(max_ticks):
    if not phase_1_done:
        # Faz 1: Keşif adımı
        executor.step(SIM_DT)
        
        # Robot dinamik hedefe vardıysa
        if executor.reached_dynamic_target:
            current = executor.dynamic_target
            if current == end_cell:
                print(f"[{k}] HEDEF BULUNDU! Keşif haritası çıkarıldı.")
                phase_1_done = True
                
                # Ulaşıldığında iç haritadan BFS ile kısayolu hesapla
                shortest_path = explorer.get_shortest_path_to_target(end_cell)
                print(f"Faz 2 (Speed Run) rotası: {shortest_path}")
                
                if len(shortest_path) < 2:
                    print("Hata: Hedefe gidebilecek tutarlı bir yol haritası çıkartılamadı.")
                    break
                
                # --- FAZ 2'YE HAZIRLIK: ROBOTU BAŞA IŞINLA ---
                print(">>> FAZ 2 (HIZLI ÇÖZÜM) BAŞLIYOR <<<")
                robot.center = Point(start_pos.x, start_pos.y) # Point nesnesini klonla
                robot.heading = initial_heading
                robot.velocity = Point(0, 0)
                
                # Faz 2: Hızlı Executor, yolu biliyor
                fast_speed = BASE_SPEED * 1.4 # %40 daha hızlı
                executor = PathExecutor(robot, maze, my_controller, path=shortest_path, 
                                        base_speed=fast_speed, 
                                        junction_threshold=JUNCTION_THRESHOLD, 
                                        lost_line_timeout=LOST_LINE_TIMEOUT)
            else:
                # Duraksamadan direkt karar ver ve yola devam et
                nxt = explorer.discover_and_decide_next(current)
                if nxt is None:
                    print(f"[{k}] Tıkandım, gidilecek yer yok! Başarısız.")
                    break
                
                executor.set_dynamic_target(nxt)
    else:
        # Faz 2: Speed Run adımı
        executor.step(SIM_DT)
        if executor.finished:
            print(f"[{k}] HEDEF BAŞARIYLA TAMAMLANDI (Speed Run Bitti)!")
            break

    # Ortak simülasyon update
    w.tick()
    w.render()
    time.sleep(SIM_DT / 4)

    # if k % 50 == 0:
    #     print(f"Adım {k} | Mode: {'Keşif' if not phase_1_done else 'SpeedRun'} | Hız: {robot.speed:.2f}")

    if w.collision_exists(robot):
        print("Çarpışma Algılandı, Simülasyon Durduruluyor!")
        break

time.sleep(2)
w.close()
