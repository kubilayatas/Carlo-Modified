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
                 lost_line_timeout: int = 10):
        self.robot = robot
        self.maze = maze_track
        self.pid = pid_controller
        self.path = path  # Liste ise Faz 2, None ise Faz 1
        self.current_path_index = 0
        self.base_speed = base_speed
        self.junction_threshold = junction_threshold
        self.lost_line_timeout = lost_line_timeout
        self._lost_counter = 0
        self.finished = False
        
        self.dynamic_target = None
        self.reached_dynamic_target = False

    def set_dynamic_target(self, cell):
        """Faz 1 (Keşif) için bir sonraki hedef hücreyi ayarla."""
        self.dynamic_target = cell
        self.reached_dynamic_target = False
        self.finished = False

    def step(self, dt: float):
        if self.finished:
            self.robot.set_control(0, -0.5)  # Dur
            return

        # Hedef hücreyi belirle
        target_cell = None
        if self.path is not None:
            if self.current_path_index + 1 < len(self.path):
                target_cell = self.path[self.current_path_index + 1]
            else:
                self.robot.set_control(0, -0.5)
                self.finished = True
                return
        else:
            if self.dynamic_target is None or self.reached_dynamic_target:
                self.robot.set_control(0, -0.5)
                return
            target_cell = self.dynamic_target

        target_pos = self.maze.cell_to_world(*target_cell)
        dx = target_pos.x - self.robot.center.x
        dy = target_pos.y - self.robot.center.y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < self.junction_threshold:
            if self.path is not None:
                self.current_path_index += 1
                if self.current_path_index >= len(self.path) - 1:
                    self.robot.set_control(0, -0.5)
                    self.finished = True
                    return
                # Sonraki hedefe geçebilmesi için yeniden target belirlemeli
                target_cell = self.path[self.current_path_index + 1]
                target_pos = self.maze.cell_to_world(*target_cell)
                dx = target_pos.x - self.robot.center.x
                dy = target_pos.y - self.robot.center.y
            else:
                self.reached_dynamic_target = True
                return

        # Sensör bazlı PID çizgi izleme
        line_error = self.robot.get_line_error()

        if line_error is not None:
            self._lost_counter = 0
            steering = self.pid.compute(line_error, dt)
            speed_factor = 1.0 - 0.5 * abs(line_error)
            throttle_target = self.base_speed * speed_factor
            current_speed = self.robot.speed
            accel = (throttle_target - current_speed) * 0.5
            self.robot.set_control(steering, accel)
        else:
            self._lost_counter += 1
            if self._lost_counter > self.lost_line_timeout:
                self.robot.set_control(0, -0.3)
            else:
                # Hedefe doğru heading düzelt
                target_angle = np.arctan2(dy, dx)
                heading_error = target_angle - self.robot.heading
                heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
                steering = np.clip(heading_error * 1.5, -1.2, 1.2)
                
                # Çizgiden koptuğu anda (keskin virajda vs.) CARLO'daki bisiklet fiziği gereği 
                # durursa yerinde dönemez, dönmesi için mutlaka ileri gitmeli!
                throttle_target = self.base_speed * 0.6  # Çizgi dışındayken %60 hızla ilerle
                accel = (throttle_target - self.robot.speed) * 0.5
                self.robot.set_control(steering, accel)

    @property
    def progress(self):
        if self.path is None: return 0.0
        if len(self.path) <= 1: return 100.0
        return 100.0 * self.current_path_index / (len(self.path) - 1)

class DynamicExplorer:
    """Faz 1: Robotun labirenti adım adım keşfederek kendi iç haritasını oluşturduğu sınıf."""
    def __init__(self, maze_track, start_cell, initial_heading_str='up'):
        self.maze = maze_track
        self.internal_graph = {}  # { (r,c): [ (nr,nc), ... ] }
        self.current_cell = start_cell
        self.heading_str = initial_heading_str
        self.visited = []

    def _get_direction_offset(self, direction):
        if direction == 'up': return (-1, 0)
        if direction == 'down': return (1, 0)
        if direction == 'left': return (0, -1)
        if direction == 'right': return (0, 1)

    def _get_left(self, d):
        return {'up':'left', 'left':'down', 'down':'right', 'right':'up'}[d]

    def _get_right(self, d):
        return {'up':'right', 'right':'down', 'down':'left', 'left':'up'}[d]

    def _get_back(self, d):
        return {'up':'down', 'down':'up', 'left':'right', 'right':'left'}[d]

    def discover_and_decide_next(self, current_cell):
        """Bu hücredeki topolojiyi alır, haritayı günceller ve Solu-İzle'ye göre sıradaki hücreyi seçer."""
        self.current_cell = current_cell
        self.visited.append(current_cell)

        # 1. Hücre topolojisini simülasyon motorundan (sensör niyetine) çek
        topology = self.maze.get_topology_at(*current_cell)
        if current_cell not in self.internal_graph:
            self.internal_graph[current_cell] = []

        neighbors = []
        for d in ['up', 'down', 'left', 'right']:
            if topology and topology.get(d, False):
                dr, dc = self._get_direction_offset(d)
                neighbors.append((current_cell[0] + dr, current_cell[1] + dc))
        
        self.internal_graph[current_cell] = neighbors

        # 2. Karar ver (Left-Hand Rule)
        candidates = [
            self._get_left(self.heading_str),
            self.heading_str,
            self._get_right(self.heading_str),
            self._get_back(self.heading_str)
        ]

        next_cell = None
        for try_dir in candidates:
            dr, dc = self._get_direction_offset(try_dir)
            target = (current_cell[0] + dr, current_cell[1] + dc)
            # Eğrisiyle doğrusuyla bu hedef komşular arasında açık (yol) mu?
            if target in neighbors:
                next_cell = target
                self.heading_str = try_dir
                break

        return next_cell

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

