"""
Keşif Stratejileri Modülü
=========================

Her strateji aynı arayüze sahiptir:
  - __init__(maze_track, start_cell, end_cell, initial_heading_str)
  - decide_next(current_cell) → next_cell veya None
  - is_phase1_done → bool (Faz 2'ye geçilmeli mi?)
  - get_phase2_path() → list  (Faz 2 için rota)
  - explored_count → int
  - name → str

Kullanım (playground.py'de):
  EXPLORATION_STRATEGY = 'full_explore'
  strategy = create_strategy(EXPLORATION_STRATEGY, maze, start_cell, end_cell, heading_str)
"""

from collections import deque


class BaseExplorer:
    """Tüm stratejiler için ortak keşif altyapısı."""

    def __init__(self, maze_track, start_cell, end_cell, initial_heading_str='up'):
        self.maze = maze_track
        self.start_cell = start_cell
        self.end_cell = end_cell
        self.internal_graph = {}  # { (r,c): [ (nr,nc), ... ] }
        self.current_cell = start_cell
        self.heading_str = initial_heading_str
        self.visited = []          # Sıralı ziyaret geçmişi
        self.visited_set = set()   # Hızlı O(1) lookup
        self.found_end = False     # Bitiş hücresi keşif sırasında bulundu mu?
        self._phase1_done = False

    @property
    def name(self) -> str:
        raise NotImplementedError

    @property
    def is_phase1_done(self) -> bool:
        return self._phase1_done

    @property
    def explored_count(self) -> int:
        return len(self.visited_set)

    def _discover_cell(self, cell):
        """Hücre topolojisini keşfet ve iç haritaya ekle."""
        if cell not in self.visited_set:
            self.visited.append(cell)
            self.visited_set.add(cell)

        topology = self.maze.get_topology_at(*cell)
        neighbors = []
        for d in ['up', 'down', 'left', 'right']:
            if topology and topology.get(d, False):
                dr, dc = self._get_direction_offset(d)
                neighbors.append((cell[0] + dr, cell[1] + dc))
        self.internal_graph[cell] = neighbors

        if cell == self.end_cell and not self.found_end:
            self.found_end = True

        return neighbors

    def decide_next(self, current_cell):
        """Alt sınıflar tarafından implement edilecek."""
        raise NotImplementedError

    def get_phase2_path(self) -> list:
        """Keşfedilen harita üzerinden BFS ile en kısa yolu hesapla."""
        queue = deque()
        queue.append(self.start_cell)
        came_from = {self.start_cell: None}

        while queue:
            curr = queue.popleft()
            if curr == self.end_cell:
                path = []
                node = self.end_cell
                while node is not None:
                    path.append(node)
                    node = came_from[node]
                return path[::-1]

            for neighbor in self.internal_graph.get(curr, []):
                if neighbor not in came_from:
                    came_from[neighbor] = curr
                    queue.append(neighbor)

        return []

    # ── Yardımcı fonksiyonlar ──

    @staticmethod
    def _get_direction_offset(direction):
        return {'up': (-1, 0), 'down': (1, 0), 'left': (0, -1), 'right': (0, 1)}[direction]

    @staticmethod
    def _offset_to_direction(dr, dc):
        if dr == -1: return 'up'
        if dr == 1: return 'down'
        if dc == -1: return 'left'
        if dc == 1: return 'right'
        return 'up'

    @staticmethod
    def _get_left(d):
        return {'up': 'left', 'left': 'down', 'down': 'right', 'right': 'up'}[d]

    @staticmethod
    def _get_right(d):
        return {'up': 'right', 'right': 'down', 'down': 'left', 'left': 'up'}[d]

    @staticmethod
    def _get_back(d):
        return {'up': 'down', 'down': 'up', 'left': 'right', 'right': 'left'}[d]


# ═══════════════════════════════════════════════════════════
# STRATEJİ 1: Sol Duvar Takibi (Left Wall Follower)
# ═══════════════════════════════════════════════════════════

class LeftWallFollower(BaseExplorer):
    """Sol duvar izleme algoritması.
    
    Robot sol elini duvara koyar ve sürekli sola dönmeye çalışır.
    Bitiş noktasına ulaştığında Faz 1 biter.
    Faz 2'de keşfedilen yolun kendisi kullanılır (optimize edilmemiş).
    """

    @property
    def name(self):
        return "Sol Duvar Takibi"

    def decide_next(self, current_cell):
        self.current_cell = current_cell
        neighbors = self._discover_cell(current_cell)

        # Bitiş noktasına vardıysa keşif biter
        if current_cell == self.end_cell:
            self._phase1_done = True
            return None

        # Sol-El Kuralı: Sol > Düz > Sağ > Geri
        candidates = [
            self._get_left(self.heading_str),
            self.heading_str,
            self._get_right(self.heading_str),
            self._get_back(self.heading_str)
        ]

        for try_dir in candidates:
            dr, dc = self._get_direction_offset(try_dir)
            target = (current_cell[0] + dr, current_cell[1] + dc)
            if target in neighbors:
                self.heading_str = try_dir
                return target

        return None  # Çıkmaz


# ═══════════════════════════════════════════════════════════
# STRATEJİ 2: Sağ Duvar Takibi (Right Wall Follower)
# ═══════════════════════════════════════════════════════════

class RightWallFollower(BaseExplorer):
    """Sağ duvar izleme algoritması — Sol duvar takibinin ayna simetrisi.
    
    Robot sağ elini duvara koyar ve sürekli sağa dönmeye çalışır.
    Bitiş noktasına ulaştığında Faz 1 biter.
    """

    @property
    def name(self):
        return "Sağ Duvar Takibi"

    def decide_next(self, current_cell):
        self.current_cell = current_cell
        neighbors = self._discover_cell(current_cell)

        if current_cell == self.end_cell:
            self._phase1_done = True
            return None

        # Sağ-El Kuralı: Sağ > Düz > Sol > Geri
        candidates = [
            self._get_right(self.heading_str),
            self.heading_str,
            self._get_left(self.heading_str),
            self._get_back(self.heading_str)
        ]

        for try_dir in candidates:
            dr, dc = self._get_direction_offset(try_dir)
            target = (current_cell[0] + dr, current_cell[1] + dc)
            if target in neighbors:
                self.heading_str = try_dir
                return target

        return None


# ═══════════════════════════════════════════════════════════
# STRATEJİ 3: Tam Keşif + En Kısa Yol (Full Explore + BFS)
# ═══════════════════════════════════════════════════════════

class FullExplorer(BaseExplorer):
    """Tüm haritayı keşfet, sonra BFS ile en kısa yolu hesapla.

    Keşif stratejisi:
    1. Ziyaret edilmemiş komşulara git (sol-el önceliğiyle)
    2. Tüm komşular ziyaret edildiyse: BFS ile en yakın frontier'a geri dön
    3. Frontier kalmayınca keşif biter, BFS ile optimal rota hesaplanır
    """

    @property
    def name(self):
        return "Tam Keşif + En Kısa Yol"

    @property
    def _has_frontier(self):
        """Keşfedilmemiş hücre var mı?"""
        for cell, neighbors in self.internal_graph.items():
            for n in neighbors:
                if n not in self.visited_set:
                    return True
        return False

    def decide_next(self, current_cell):
        self.current_cell = current_cell
        neighbors = self._discover_cell(current_cell)

        # Keşif tamamlandı mı?
        if not self._has_frontier:
            self._phase1_done = True
            return None

        # Öncelik 1: Ziyaret edilmemiş komşu (sol-el sırasıyla)
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

        # Öncelik 2: Geri dön — en yakın frontier'a BFS yolu
        path_to_frontier = self._find_path_to_frontier(current_cell)
        if path_to_frontier and len(path_to_frontier) > 1:
            next_step = path_to_frontier[1]
            dr = next_step[0] - current_cell[0]
            dc = next_step[1] - current_cell[1]
            self.heading_str = self._offset_to_direction(dr, dc)
            return next_step

        # Gerçekten tıkandı
        self._phase1_done = True
        return None

    def _find_path_to_frontier(self, start):
        """BFS ile ziyaret edilmiş hücreler üzerinden en yakın frontier'a yol bul."""
        queue = deque([(start, [start])])
        seen = {start}

        while queue:
            cell, path = queue.popleft()
            for n in self.internal_graph.get(cell, []):
                if n not in self.visited_set:
                    return path
            for n in self.internal_graph.get(cell, []):
                if n in self.visited_set and n not in seen:
                    seen.add(n)
                    queue.append((n, path + [n]))

        return None


# ═══════════════════════════════════════════════════════════
# FABRİKA FONKSİYONU
# ═══════════════════════════════════════════════════════════

STRATEGIES = {
    'left_wall':    LeftWallFollower,
    'right_wall':   RightWallFollower,
    'full_explore': FullExplorer,
}

def create_strategy(name: str, maze_track, start_cell, end_cell, initial_heading_str='up'):
    """Strateji adına göre keşif nesnesi oluştur.
    
    Kullanılabilir stratejiler:
      - 'left_wall'    : Sol Duvar Takibi
      - 'right_wall'   : Sağ Duvar Takibi
      - 'full_explore' : Tam Keşif + En Kısa Yol (BFS)
    """
    if name not in STRATEGIES:
        available = ', '.join(f"'{k}'" for k in STRATEGIES)
        raise ValueError(f"Bilinmeyen strateji: '{name}'. Kullanılabilir: {available}")

    return STRATEGIES[name](maze_track, start_cell, end_cell, initial_heading_str)
