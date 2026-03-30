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
    """Planlanan hücre yolunu gerçek zamanlı robot kontrolüne dönüştürür."""

    # Hücre geçiş yönleri → heading açısı (radyan)
    HEADING_MAP = {
        'up':    np.pi / 2,
        'down':  3 * np.pi / 2,   # veya -np.pi/2
        'left':  np.pi,
        'right': 0.0
    }

    def __init__(self, robot, maze_track, pid_controller, path: list,
                 base_speed: float = 3.0,
                 junction_threshold: float = 2.0,
                 lost_line_timeout: int = 500):
        """
        robot: LineFollowerRobot referansı
        maze_track: MazeTrack referansı
        pid_controller: PIDController referansı
        path: [(r,c), ...] planlanan hücre yolu
        base_speed: Düz çizgi hızı (m/s)
        junction_threshold: Kavşağa yakınlık eşiği (metre)
        lost_line_timeout: Çizgi kaybında kaç tick bekle
        """
        self.robot = robot
        self.maze = maze_track
        self.pid = pid_controller
        self.path = path
        self.current_path_index = 0
        self.base_speed = base_speed
        self.junction_threshold = junction_threshold
        self.lost_line_timeout = lost_line_timeout
        self._lost_counter = 0
        self.finished = False

    def _get_direction(self, from_cell, to_cell):
        """İki hücre arasındaki yönü belirle."""
        dr = to_cell[0] - from_cell[0]
        dc = to_cell[1] - from_cell[1]
        if dr == -1: return 'up'
        if dr == 1:  return 'down'
        if dc == -1: return 'left'
        if dc == 1:  return 'right'
        return None

    def _target_heading(self):
        """Şu anki hedef heading açısını döndür."""
        if self.current_path_index + 1 >= len(self.path):
            return None
        d = self._get_direction(self.path[self.current_path_index],
                                self.path[self.current_path_index + 1])
        return self.HEADING_MAP.get(d) if d else None

    def step(self, dt: float):
        """Her simülasyon tick'inde çağrılır.
        Robot kontrolünü ayarlar (set_control)."""
        if self.finished or self.current_path_index >= len(self.path) - 1:
            self.robot.set_control(0, -0.5)  # Dur
            self.finished = True
            return

        # Mevcut hedef hücre
        target_cell = self.path[self.current_path_index + 1]
        target_pos = self.maze.cell_to_world(*target_cell)

        # Hedefe olan mesafe
        dx = target_pos.x - self.robot.center.x
        dy = target_pos.y - self.robot.center.y
        dist = np.sqrt(dx**2 + dy**2)

        # Hedef hücreye yeterince yaklaştıysa sonraki hücreye geç
        if dist < self.junction_threshold:
            self.current_path_index += 1
            if self.current_path_index >= len(self.path) - 1:
                self.robot.set_control(0, -0.5)
                self.finished = True
                return

        # Sensör bazlı PID çizgi izleme
        line_error = self.robot.get_line_error()

        if line_error is not None:
            self._lost_counter = 0
            steering = self.pid.compute(line_error, dt)
            # Dönüşlerde hız azalt
            speed_factor = 1.0 - 0.5 * abs(line_error)
            throttle_target = self.base_speed * speed_factor
            current_speed = self.robot.speed
            accel = (throttle_target - current_speed) * 0.5
            self.robot.set_control(steering, accel)
        else:
            # Çizgi kayıp — son bilinen yöne doğru yavaşça ilerle
            self._lost_counter += 1
            if self._lost_counter > self.lost_line_timeout:
                # Çok uzun süre kayıp — dur
                self.robot.set_control(0, -0.3)
            else:
                # Hedefe doğru heading düzelt
                target_angle = np.arctan2(dy, dx)
                heading_error = target_angle - self.robot.heading
                # Normalize [-pi, pi]
                heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
                steering = np.clip(heading_error * 0.3, -0.5, 0.5)
                self.robot.set_control(steering, 0.05)

    @property
    def progress(self):
        """Yüzde ilerleme."""
        if len(self.path) <= 1:
            return 100.0
        return 100.0 * self.current_path_index / (len(self.path) - 1)
