from collections import deque
import numpy as np

class MazePlanner:
    """Grid üzerinde yol planlama."""

    def __init__(self, maze_track):
        """maze_track: MazeTrack nesnesi."""
        self.maze = maze_track
        self.adj = maze_track.get_adjacency()

    def bfs(self, start: tuple, end: tuple) -> list:
        """BFS ile en kısa yolu bul.
        Dönüş: [(r,c), ...] hücre listesi (start dahil, end dahil)."""
        queue = deque()
        queue.append(start)
        visited = {start: None}

        while queue:
            current = queue.popleft()
            if current == end:
                # Yolu geri sar
                path = []
                node = end
                while node is not None:
                    path.append(node)
                    node = visited[node]
                return path[::-1]

            for neighbor in self.adj.get(current, []):
                if neighbor not in visited:
                    visited[neighbor] = current
                    queue.append(neighbor)

        return []  # Yol bulunamadı

    def flood_fill(self, end: tuple) -> dict:
        """Flood fill: her hücreden bitiş noktasına olan mesafeyi hesapla.
        Dönüş: dict { (r,c): distance }"""
        distances = {end: 0}
        queue = deque([end])

        while queue:
            current = queue.popleft()
            for neighbor in self.adj.get(current, []):
                if neighbor not in distances:
                    distances[neighbor] = distances[current] + 1
                    queue.append(neighbor)

        return distances

    def flood_fill_path(self, start: tuple, end: tuple) -> list:
        """Flood fill mesafe haritasını kullanarak greedy yol çıkar."""
        distances = self.flood_fill(end)
        if start not in distances:
            return []

        path = [start]
        current = start
        while current != end:
            neighbors = self.adj.get(current, [])
            # En düşük mesafeli komşuyu seç
            best = min(neighbors, key=lambda n: distances.get(n, float('inf')))
            path.append(best)
            current = best

        return path

    def left_wall_follower(self, start: tuple, end: tuple, initial_direction: str = 'up') -> list:
        """Sol duvar izleme algoritması.
        initial_direction: robotun başlangıçtaki yönü ('up', 'down', 'left', 'right')
        Dönüş: [(r,c), ...] ziyaret sırası (tekrarlar olabilir)."""

        # Yön vektörleri: (dr, dc)
        DIR_MAP = {
            'up':    (-1, 0),
            'right': (0, 1),
            'down':  (1, 0),
            'left':  (0, -1)
        }
        # Sol dönüş sırası: up → left → down → right → up
        LEFT_TURN = {'up': 'left', 'left': 'down', 'down': 'right', 'right': 'up'}
        RIGHT_TURN = {'up': 'right', 'right': 'down', 'down': 'left', 'left': 'up'}

        direction = initial_direction
        current = start
        path = [current]
        max_steps = self.maze.rows * self.maze.cols * 4  # Sonsuz döngü koruması

        for _ in range(max_steps):
            if current == end:
                break

            # Sol duvar izleme: önce sola dönmeyi dene, sonra düz, sonra sağa, sonra geri
            left_dir = LEFT_TURN[direction]
            candidates = [left_dir, direction, RIGHT_TURN[direction], LEFT_TURN[LEFT_TURN[direction]]]

            moved = False
            for try_dir in candidates:
                dr, dc = DIR_MAP[try_dir]
                nr, nc = current[0] + dr, current[1] + dc
                if (nr, nc) in self.adj.get(current, []):
                    direction = try_dir
                    current = (nr, nc)
                    path.append(current)
                    moved = True
                    break

            if not moved:
                break  # Tıkandı

        return path

    def right_wall_follower(self, start: tuple, end: tuple, initial_direction: str = 'up') -> list:
        """Sağ duvar izleme — sol duvar izlemenin ayna simetrisi."""
        DIR_MAP = {
            'up':    (-1, 0),
            'right': (0, 1),
            'down':  (1, 0),
            'left':  (0, -1)
        }
        LEFT_TURN = {'up': 'left', 'left': 'down', 'down': 'right', 'right': 'up'}
        RIGHT_TURN = {'up': 'right', 'right': 'down', 'down': 'left', 'left': 'up'}

        direction = initial_direction
        current = start
        path = [current]
        max_steps = self.maze.rows * self.maze.cols * 4

        for _ in range(max_steps):
            if current == end:
                break

            right_dir = RIGHT_TURN[direction]
            candidates = [right_dir, direction, LEFT_TURN[direction], RIGHT_TURN[RIGHT_TURN[direction]]]

            moved = False
            for try_dir in candidates:
                dr, dc = DIR_MAP[try_dir]
                nr, nc = current[0] + dr, current[1] + dc
                if (nr, nc) in self.adj.get(current, []):
                    direction = try_dir
                    current = (nr, nc)
                    path.append(current)
                    moved = True
                    break

            if not moved:
                break

        return path

class PathExecutor:
    """Planlanan tam yolu veya dinamik anlık hedefi gerçek zamanlı robot kontrolüne dönüştürür."""

    # Hücre geçiş yönleri → heading açısı (radyan)
    HEADING_MAP = {
        'up':    np.pi / 2,
        'down':  3 * np.pi / 2,
        'left':  np.pi,
        'right': 0.0
    }

    def __init__(self, robot, maze_track, pid_controller, path=None,
                 base_speed: float = 3.0,
                 junction_threshold: float = 2.0,
                 lost_line_timeout: int = 10,
                 junction_pause_ticks: int = 15,
                 turn_in_place_threshold: float = None,
                 turn_speed: float = 0.08):
        self.robot = robot
        self.maze = maze_track
        self.pid = pid_controller
        self.path = path  # Liste ise Faz 2, None ise Faz 1
        self.current_path_index = 0
        self.base_speed = base_speed
        self.junction_threshold = junction_threshold
        self.lost_line_timeout = lost_line_timeout
        self.junction_pause_ticks = junction_pause_ticks
        self._lost_counter = 0
        self.finished = False
        
        self.dynamic_target = None
        self.reached_dynamic_target = False

        # Kavşak bekleme durumu
        self._pause_counter = 0
        self._paused_at_cell = None

        # ── 2WD Yerinde Dönüş (Pivot Turn) ayarları ──
        # turn_in_place_threshold: Bu açıdan (rad) büyük heading farkında
        # robot yerinde döner. Varsayılan π/4 = 45°
        self.turn_in_place_threshold = turn_in_place_threshold if turn_in_place_threshold is not None else np.pi / 4
        # turn_speed: Yerinde dönüş sırasında tekerlek hızı (m/s)
        self.turn_speed = turn_speed
        # Heading toleransı: pivot dönüş bu kadar yakınlaşınca biter (~8°)
        self.turn_complete_tolerance = 0.15
        # Dönüş durumu (loglama için)
        self._is_turning = False

    def set_dynamic_target(self, cell):
        """Faz 1 (Keşif) için bir sonraki hedef hücreyi ayarla."""
        self.dynamic_target = cell
        self.reached_dynamic_target = False
        self.finished = False

    @property
    def is_paused(self):
        """Kavşakta bekleme durumunda mı?"""
        return self._pause_counter > 0

    @property
    def is_turning(self):
        """Yerinde dönüş yapıyor mu?"""
        return self._is_turning

    @property
    def paused_at(self):
        """Hangi hücrede bekliyor (None ise beklemiyor)."""
        return self._paused_at_cell if self.is_paused else None

    def step(self, dt: float):
        if self.finished:
            self.robot.set_control(0, 0)  # Dur
            return

        # ── Kavşakta bekleme durumu ──
        if self._pause_counter > 0:
            self.robot.set_control(0, 0)  # Yerinde dur
            self._pause_counter -= 1
            if self._pause_counter == 0:
                # Bekleme bitti → kavşak kararını uygula
                self._paused_at_cell = None
                if self.path is not None:
                    # Faz 2: Bir sonraki hücreye geç
                    self.current_path_index += 1
                    if self.current_path_index >= len(self.path) - 1:
                        self.finished = True
                else:
                    # Faz 1: Hedefe vardığını bildir
                    self.reached_dynamic_target = True
            return

        # Hedef hücreyi belirle
        target_cell = None
        if self.path is not None:
            if self.current_path_index + 1 < len(self.path):
                target_cell = self.path[self.current_path_index + 1]
            else:
                self.robot.set_control(0, 0)
                self.finished = True
                return
        else:
            if self.dynamic_target is None or self.reached_dynamic_target:
                self.robot.set_control(0, 0)
                return
            target_cell = self.dynamic_target

        target_pos = self.maze.cell_to_world(*target_cell)
        dx = target_pos.x - self.robot.center.x
        dy = target_pos.y - self.robot.center.y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < self.junction_threshold:
            # ── Kavşağa vardık → dur ve bekle ──
            self.robot.set_control(0, 0)
            self._paused_at_cell = target_cell
            self._is_turning = False
            if self.junction_pause_ticks > 0:
                self._pause_counter = self.junction_pause_ticks
                return
            else:
                # Bekleme süresi 0 ise anında geç (eski davranış)
                self._paused_at_cell = None
                if self.path is not None:
                    self.current_path_index += 1
                    if self.current_path_index >= len(self.path) - 1:
                        self.finished = True
                        return
                    target_cell = self.path[self.current_path_index + 1]
                    target_pos = self.maze.cell_to_world(*target_cell)
                    dx = target_pos.x - self.robot.center.x
                    dy = target_pos.y - self.robot.center.y
                else:
                    self.reached_dynamic_target = True
                    return

        # ── 2WD Yerinde Dönüş (Pivot Turn) ──
        # Hedefe doğru heading farkı büyükse, ileri gitmeden önce yerinde dön.
        # Bu diferansiyel sürüşün en büyük avantajı: yerinde dönebilmek!
        target_angle = np.arctan2(dy, dx)
        heading_error = target_angle - self.robot.heading
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        if abs(heading_error) > self.turn_in_place_threshold:
            # Yerinde pivot dönüşü: tekerlekler zıt yönde döner
            self._is_turning = True
            ts = self.turn_speed
            if heading_error > 0:
                self.robot.set_control(-ts, ts)   # Sola dön (CCW)
            else:
                self.robot.set_control(ts, -ts)   # Sağa dön (CW)
            return

        # Pivot dönüşü bitti veya gerekmedi
        if self._is_turning:
            self._is_turning = False

        # ── Sensör bazlı PID çizgi izleme (Diferansiyel Sürüş) ──
        line_error = self.robot.get_line_error()
        L = self.robot.track_width

        if line_error is not None:
            self._lost_counter = 0
            # PID çıktısı artık açısal hız (omega) olarak yorumlanıyor
            omega = self.pid.compute(line_error, dt)
            speed_factor = 1.0 - 0.5 * abs(line_error)
            base = self.base_speed * speed_factor
            v_R = base + omega * L / 2
            v_L = base - omega * L / 2
            v_R = np.clip(v_R, self.robot.min_speed, self.robot.max_speed)
            v_L = np.clip(v_L, self.robot.min_speed, self.robot.max_speed)
            self.robot.set_control(v_L, v_R)
        else:
            self._lost_counter += 1
            if self._lost_counter > self.lost_line_timeout:
                self.robot.set_control(0, 0)
            else:
                # Hedefe doğru heading düzelt (küçük açı — PID yeterli)
                omega = np.clip(heading_error * 2.0, -5.0, 5.0)
                base = self.base_speed * 0.5
                v_R = base + omega * L / 2
                v_L = base - omega * L / 2
                v_R = np.clip(v_R, -self.robot.max_speed, self.robot.max_speed)
                v_L = np.clip(v_L, -self.robot.max_speed, self.robot.max_speed)
                self.robot.set_control(v_L, v_R)

    @property
    def progress(self):
        if self.path is None: return 0.0
        if len(self.path) <= 1: return 100.0
        return 100.0 * self.current_path_index / (len(self.path) - 1)

class DynamicExplorer:
    """Faz 1: Robotun labirenti TAMAMEN keşfederek iç haritasını oluşturduğu sınıf.
    
    Keşif stratejisi:
    1. Öncelik: Ziyaret edilmemiş komşulara git (sol-el kuralı sırasıyla)
    2. Tüm komşular ziyaret edildiyse: BFS ile en yakın 'frontier' hücresine geri dön
    3. Hiç frontier kalmadığında: Keşif tamamlandı
    """

    def __init__(self, maze_track, start_cell, initial_heading_str='up'):
        self.maze = maze_track
        self.internal_graph = {}  # { (r,c): [ (nr,nc), ... ] }
        self.current_cell = start_cell
        self.heading_str = initial_heading_str
        self.visited = []          # Sıralı ziyaret geçmişi (uyumluluk için)
        self.visited_set = set()   # Hızlı O(1) lookup
        self.found_end = False     # Bitiş hücresi keşif sırasında bulundu mu?

    @property
    def is_exploration_complete(self):
        """Tüm erişilebilir hücreler ziyaret edildi mi?"""
        for cell, neighbors in self.internal_graph.items():
            for n in neighbors:
                if n not in self.visited_set:
                    return False
        return len(self.internal_graph) > 0  # En az 1 hücre keşfedilmiş olmalı

    @property
    def explored_count(self):
        """Kaç hücre keşfedildi?"""
        return len(self.visited_set)

    def _get_direction_offset(self, direction):
        if direction == 'up': return (-1, 0)
        if direction == 'down': return (1, 0)
        if direction == 'left': return (0, -1)
        if direction == 'right': return (0, 1)

    def _offset_to_direction(self, dr, dc):
        """(dr, dc) offset'inden yön string'ine dönüştür."""
        if dr == -1: return 'up'
        if dr == 1: return 'down'
        if dc == -1: return 'left'
        if dc == 1: return 'right'
        return self.heading_str

    def _get_left(self, d):
        return {'up':'left', 'left':'down', 'down':'right', 'right':'up'}[d]

    def _get_right(self, d):
        return {'up':'right', 'right':'down', 'down':'left', 'left':'up'}[d]

    def _get_back(self, d):
        return {'up':'down', 'down':'up', 'left':'right', 'right':'left'}[d]

    def discover_and_decide_next(self, current_cell):
        """Bu hücreyi keşfet, haritayı güncelle ve bir sonraki hedefi seç.
        
        Öncelik sırası:
        1. Ziyaret edilmemiş komşu (sol-el kuralı sırasıyla)
        2. Ziyaret edilmiş komşular üzerinden BFS ile en yakın frontier'a geri dön
        3. None → keşif tamamlandı
        """
        self.current_cell = current_cell

        # Ziyaret kaydı (ilk kez ziyaret ediyorsak)
        if current_cell not in self.visited_set:
            self.visited.append(current_cell)
            self.visited_set.add(current_cell)

        # 1. Hücre topolojisini simülasyon motorundan (sensör niyetine) çek
        topology = self.maze.get_topology_at(*current_cell)

        neighbors = []
        for d in ['up', 'down', 'left', 'right']:
            if topology and topology.get(d, False):
                dr, dc = self._get_direction_offset(d)
                neighbors.append((current_cell[0] + dr, current_cell[1] + dc))
        
        self.internal_graph[current_cell] = neighbors

        # Keşif tamamlandı mı kontrol et
        if self.is_exploration_complete:
            return None

        # 2. Öncelik 1: Ziyaret edilmemiş komşuya git (sol-el sırasıyla)
        candidates = [
            self._get_left(self.heading_str),
            self.heading_str,
            self._get_right(self.heading_str),
            self._get_back(self.heading_str)
        ]

        for try_dir in candidates:
            dr, dc = self._get_direction_offset(try_dir)
            target = (current_cell[0] + dr, current_cell[1] + dc)
            if target in neighbors and target not in self.visited_set:
                self.heading_str = try_dir
                return target

        # 3. Öncelik 2: Tüm komşular ziyaret edilmiş → en yakın frontier'a geri dön
        path_to_frontier = self._find_path_to_frontier(current_cell)
        if path_to_frontier and len(path_to_frontier) > 1:
            next_step = path_to_frontier[1]
            dr = next_step[0] - current_cell[0]
            dc = next_step[1] - current_cell[1]
            self.heading_str = self._offset_to_direction(dr, dc)
            return next_step

        return None  # Gerçekten tıkandı veya keşif tamam

    def _find_path_to_frontier(self, start):
        """BFS ile internal_graph üzerinde en yakın 'ziyaret edilmemiş komşusu olan'
        hücreye giden yolu bul. Sadece ziyaret edilmiş hücreler üzerinden geçer."""
        from collections import deque
        queue = deque([(start, [start])])
        seen = {start}

        while queue:
            cell, path = queue.popleft()

            # Bu hücrenin ziyaret edilmemiş komşusu var mı?
            for n in self.internal_graph.get(cell, []):
                if n not in self.visited_set:
                    return path  # Bu hücreye git (oradan ziyaret edilmemiş komşuya geçecek)

            # Ziyaret edilmiş komşulara devam et
            for n in self.internal_graph.get(cell, []):
                if n in self.visited_set and n not in seen:
                    seen.add(n)
                    queue.append((n, path + [n]))

        return None

    def get_shortest_path_to_target(self, end_cell):
        """Keşfedilen internal_graph üzerinden BFS ile en kısa rotayı çıkarır."""
        from collections import deque
        queue = deque()
        start = self.visited[0]
        queue.append(start)
        visited = {start: None}

        while queue:
            curr = queue.popleft()
            if curr == end_cell:
                path = []
                node = end_cell
                while node is not None:
                    path.append(node)
                    node = visited[node]
                return path[::-1]

            for neighbor in self.internal_graph.get(curr, []):
                if neighbor not in visited:
                    visited[neighbor] = curr
                    queue.append(neighbor)
        
        return []

