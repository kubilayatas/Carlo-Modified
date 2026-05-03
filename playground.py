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
from maze_solver import PathExecutor
from exploration_strategies import create_strategy
from maze_presets import MAZE_MEDIUM_T, MAZE_EXPERT, MAZE_MineOne, MAZE_SIMPLE_S, MAZE_HARD_DEADENDS, MAZE_30x30

# ==========================================================
# 1. KEŞİF ALGORİTMASI SEÇİMİ (EXPLORATION STRATEGY)
# ==========================================================
# Kullanılabilir stratejiler:
#   'left_wall'    → Sol Duvar Takibi (bitiş noktasını bulunca durur)
#   'right_wall'   → Sağ Duvar Takibi (bitiş noktasını bulunca durur)
#   'full_explore' → Tüm Haritayı Keşfet + En Kısa Yol (BFS)

EXPLORATION_STRATEGY = 'full_explore'

# ==========================================================
# 2. ÇİZGİ İZLEME KONTROLCÜLERİ (STEERING CONTROLLERS)
# ==========================================================
class BangBangController:
    """ Basit "Eğer soldaysa Sağa Dön, Sağdaysa Sola Dön" kontrolcüsü.
    Çıktı artık açısal hız (omega, rad/s) olarak yorumlanıyor. """
    def compute(self, line_error, dt):
        # line_error değeri -1.0 (en sol) ile +1.0 (en sağ) arasında gelir.
        if line_error < -0.1:
            return -3.0  # Tam Sola dön (ω = -3 rad/s)
        elif line_error > 0.1:
            return 3.0   # Tam Sağa dön (ω = +3 rad/s)
        else:
            return 0.0   # Düz git

class CustomPIDController:
    """ Standart PID Algoritması (maze_solver içindekiyle aynı arayüz) """
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
        return max(-5.0, min(5.0, output)) # Limit kontrolü — omega (rad/s) olarak geniş aralık


# *** KULLANILACAK KONTROLCÜYÜ BURADAN SEÇİN ***
#my_controller = BangBangController()
my_controller = CustomPIDController(kp=2.0, ki=0.001, kd=0.2)

# ==========================================================
# 3. SİMÜLASYON AYARLARI (HARİTA VE FİZİK)
# ==========================================================
MAZE_GRID = MAZE_30x30       # Seçilen Harita: MAZE_MineOne, MAZE_MEDIUM_T vb.
BASE_SPEED = 0.15             # Robotun Düz Yol Hızı (m/s) — L virajlarda çizgiyi kaçırmaması için düşük
CELL_SIZE = 0.1               # Her Bir Labirent Karesinin Boyutu (Metre)
SIM_DT = 0.05                 # Zaman Adımı

# Robot ve Sensör Ayarları
ROBOT_SENSOR_COUNT = 8        # IR Sensör Sayısı
SENSOR_SPREAD = 0.056         # Sensör dizilimi genişliği (Metre, çok açıksa çizgiyi görmeyebilir)
SENSOR_OFFSET = 0.08          # Sensörlerin robot merkezinden ne kadar önde olduğu (Metre)

# Çizgi ve Navigasyon Ayarları
LINE_WIDTH = 0.02             # Yerdeki çizginin kalınlığı (Metre)
JUNCTION_THRESHOLD = 0.05     # Kavşak merkezine varış kabul edilme hassasiyeti (Metre)
LOST_LINE_TIMEOUT = 500       # Çizgi kaybedildiğinde kaç adım boyunca hedefe yönelmeye çalışacak
JUNCTION_PAUSE_TICKS = 0     # Kavşağa varınca kaç tick bekleyecek (dur-düşün-git)



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

# ── Keşif stratejisini oluştur ──
strategy = create_strategy(EXPLORATION_STRATEGY, maze, start_cell, end_cell, initial_heading_str)

# Robotu dünyaya ekle
robot = LineFollowerRobot(start_pos, heading=initial_heading, sensor_count=ROBOT_SENSOR_COUNT,
                          sensor_spread=SENSOR_SPREAD, sensor_forward_offset=SENSOR_OFFSET)
robot.velocity = Point(0, 0)
w.add(robot)
robot.attach_to_world(w)

# Sürücü (Faz 1 için path önceden verilmez)
executor = PathExecutor(robot, maze, my_controller, path=None, base_speed=BASE_SPEED,
                        junction_threshold=JUNCTION_THRESHOLD, lost_line_timeout=LOST_LINE_TIMEOUT,
                        junction_pause_ticks=JUNCTION_PAUSE_TICKS)

# İlk dinamik hücre hedefini stratejiden al:
next_cell = strategy.decide_next(start_cell)
if next_cell:
    executor.set_dynamic_target(next_cell)

# GUI ve Döngü
w.render()
max_ticks = 10000  # Tam keşif daha uzun sürebilir
phase_1_done = False

print(f">>> FAZ 1 ({strategy.name.upper()}) BAŞLADI <<<")
for k in range(max_ticks):
    if not phase_1_done:
        # Faz 1: Keşif adımı
        executor.step(SIM_DT)

        # Kavşakta bekliyorsa logla
        if executor.is_paused and executor.paused_at is not None:
            if executor._pause_counter == executor.junction_pause_ticks - 1:
                cell = executor.paused_at
                # Çıkmaz sokak tespiti: sadece 1 komşusu olan hücre
                neighbors = strategy.internal_graph.get(cell, [])
                if len(neighbors) == 1:
                    print(f"  [{k}] ✗ ÇIKMAZ SOKAK {cell}! Geri dönüyorum...")
                else:
                    print(f"  [{k}] Kavşak {cell}'a vardım, düşünüyorum...")

        # Robot dinamik hedefe vardıysa (bekleme bittikten sonra tetiklenir)
        if executor.reached_dynamic_target:
            current = executor.dynamic_target

            # Bitiş noktası bulunduğunda logla
            if current == end_cell and not strategy.found_end:
                strategy.found_end = True
                print(f"  [{k}] ★ Bitiş noktası ({end_cell}) bulundu!")
                if strategy.is_phase1_done:
                    print(f"       Strateji gereği keşif burada bitiyor.")

            # Stratejiden bir sonraki hedefi al
            nxt = strategy.decide_next(current)
            if nxt is None or strategy.is_phase1_done:
                # Keşif tamamlandı!
                print(f"\n[{k}] ═══ KEŞİF TAMAMLANDI ({strategy.name}) ═══")
                print(f"  Keşfedilen hücre sayısı: {strategy.explored_count}")
                print(f"  Bitiş noktası {'bulundu ✓' if strategy.found_end else 'BULUNAMADI ✗'}")
                phase_1_done = True

                if not strategy.found_end:
                    print("Hata: Bitiş noktasına erişilemedi!")
                    break

                # Keşfedilen harita üzerinden en kısa yolu hesapla
                shortest_path = strategy.get_phase2_path()
                print(f"  Optimal rota ({len(shortest_path)} adım): {shortest_path}")

                if len(shortest_path) < 2:
                    print("Hata: Hedefe gidebilecek tutarlı bir yol haritası çıkartılamadı.")
                    break

                # --- FAZ 2'YE HAZIRLIK: ROBOTU BAŞA IŞINLA ---
                print("\n>>> FAZ 2 (HIZLI ÇÖZÜM) BAŞLIYOR <<<")
                robot.center = Point(start_pos.x, start_pos.y)
                robot.heading = initial_heading
                robot.velocity = Point(0, 0)

                # Faz 2: Hızlı Executor, yolu biliyor
                fast_speed = BASE_SPEED * 1.4
                executor = PathExecutor(robot, maze, my_controller, path=shortest_path,
                                        base_speed=fast_speed,
                                        junction_threshold=JUNCTION_THRESHOLD,
                                        lost_line_timeout=LOST_LINE_TIMEOUT,
                                        junction_pause_ticks=JUNCTION_PAUSE_TICKS)
            else:
                print(f"  [{k}] Karar: {current} → {nxt}  (keşfedilen: {strategy.explored_count})")
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

    if w.collision_exists(robot):
        print("Çarpışma Algılandı, Simülasyon Durduruluyor!")
        break

time.sleep(2)
w.close()
